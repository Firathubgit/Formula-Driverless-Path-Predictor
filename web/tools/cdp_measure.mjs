// Minimal Chrome DevTools Protocol driver: navigate, wait, evaluate, print.
//   node web/tools/cdp_measure.mjs <port> <url> <jsExpression> [waitMs]
// Needs a Chrome already running with --remote-debugging-port=<port>.
const [port, url, expr, waitMsArg] = process.argv.slice(2);
const waitMs = Number(waitMsArg || 4000);

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function findPage(port) {
  for (let i = 0; i < 40; i++) {
    try {
      const list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
      const page = list.find((t) => t.type === 'page' && t.webSocketDebuggerUrl);
      if (page) return page;
    } catch { /* not up yet */ }
    await sleep(250);
  }
  throw new Error('no debuggable page on port ' + port);
}

const page = await findPage(port);
const ws = new WebSocket(page.webSocketDebuggerUrl);
let id = 0;
const pending = new Map();
const events = [];

ws.addEventListener('message', (ev) => {
  const msg = JSON.parse(ev.data);
  if (msg.id && pending.has(msg.id)) { pending.get(msg.id)(msg); pending.delete(msg.id); }
  else if (msg.method) events.push(msg);
});
await new Promise((res, rej) => {
  ws.addEventListener('open', res, { once: true });
  ws.addEventListener('error', rej, { once: true });
});

const send = (method, params = {}) => new Promise((res, rej) => {
  const myId = ++id;
  pending.set(myId, (msg) => (msg.error ? rej(new Error(method + ': ' + JSON.stringify(msg.error))) : res(msg.result)));
  ws.send(JSON.stringify({ id: myId, method, params }));
  setTimeout(() => { if (pending.has(myId)) { pending.delete(myId); rej(new Error('timeout: ' + method)); } }, 120000);
});

await send('Page.enable');
await send('Runtime.enable');
try {
  await send('Network.enable');
  await send('Network.setCacheDisabled', { cacheDisabled: true });
} catch { /* older builds */ }
await send('Page.navigate', { url });

// wait for the load event, then the extra settle time
const deadline = Date.now() + 30000;
while (Date.now() < deadline && !events.some((e) => e.method === 'Page.loadEventFired')) await sleep(100);
await sleep(waitMs);

const res = await send('Runtime.evaluate', {
  expression: expr, awaitPromise: true, returnByValue: true, timeout: 110000,
});
if (res.exceptionDetails) {
  console.log('EVAL EXCEPTION:', JSON.stringify(res.exceptionDetails.exception && res.exceptionDetails.exception.description || res.exceptionDetails));
} else {
  console.log(typeof res.result.value === 'string' ? res.result.value : JSON.stringify(res.result.value, null, 1));
}
ws.close();
process.exit(0);
