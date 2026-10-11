// Capture a screenshot of an ALREADY-LOADED page (launch Chrome with the URL).
//   node web/tools/cdp_shot.mjs <port> <setupJs> <waitMs> <outPng>
// Avoids Page.enable/navigate entirely: only Runtime.evaluate and Page.captureScreenshot.
import fs from 'node:fs';

const [port, setupJs, waitMsArg, out] = process.argv.slice(2);
const waitMs = Number(waitMsArg || 3000);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function findPage() {
  for (let i = 0; i < 60; i++) {
    try {
      const list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
      const page = list.find((t) => t.type === 'page' && t.webSocketDebuggerUrl
        && !t.url.startsWith('chrome://') && !t.url.startsWith('devtools://'));
      if (page) return page;
    } catch { /* not up yet */ }
    await sleep(250);
  }
  throw new Error('no debuggable page on port ' + port);
}

const page = await findPage();
console.log('target:', page.url);
const ws = new WebSocket(page.webSocketDebuggerUrl);
let id = 0;
const pending = new Map();
ws.addEventListener('message', (ev) => {
  const m = JSON.parse(ev.data);
  if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
});
ws.addEventListener('close', () => console.error('!! websocket closed'));
ws.addEventListener('error', (e) => console.error('!! websocket error', e.message || e));
await new Promise((res, rej) => {
  ws.addEventListener('open', res, { once: true });
  ws.addEventListener('error', rej, { once: true });
});
console.log('ws open');

const send = (method, params = {}, ms = 30000) => new Promise((res, rej) => {
  const myId = ++id;
  pending.set(myId, (m) => (m.error ? rej(new Error(method + ': ' + JSON.stringify(m.error))) : res(m.result)));
  ws.send(JSON.stringify({ id: myId, method, params }));
  setTimeout(() => { if (pending.has(myId)) { pending.delete(myId); rej(new Error('timeout ' + method)); } }, ms);
});

// wait for the app to finish building
for (let i = 0; i < 60; i++) {
  const r = await send('Runtime.evaluate', { expression: 'typeof window.__fd', returnByValue: true });
  if (r.result.value === 'object') break;
  await sleep(500);
}

if (setupJs && setupJs.trim()) {
  const r = await send('Runtime.evaluate', { expression: setupJs, awaitPromise: true, returnByValue: true }, 90000);
  if (r.exceptionDetails) console.error('setup threw:', JSON.stringify(r.exceptionDetails.exception?.description || r.exceptionDetails));
  else {
    const value = r.result.value;
    // A setup that returns a data URL is itself the picture: reading the WebGL canvas back in
    // the same task as the render is reliable, where Page.captureScreenshot can hang on a page
    // that never stops animating.
    if (typeof value === 'string' && value.startsWith('data:image/')) {
      fs.writeFileSync(out, Buffer.from(value.split(',')[1], 'base64'));
      console.log('wrote', out, fs.statSync(out).size, 'bytes');
      ws.close();
      process.exit(0);
    }
    console.log('setup ->', JSON.stringify(value));
  }
}
await sleep(waitMs);

const shot = await send('Page.captureScreenshot', { format: 'png' }, 60000);
fs.writeFileSync(out, Buffer.from(shot.data, 'base64'));
console.log('wrote', out, fs.statSync(out).size, 'bytes');
ws.close();
process.exit(0);
