(async () => {
  const errors = window.__errors || [];
  window.addEventListener('error', (e) => errors.push('error: ' + e.message));
  const overlay = document.getElementById('overlay');
  const status = document.getElementById('status');
  const gpuNote = document.getElementById('gpuNote');
  overlay.classList.add('hidden');           // dismiss the start overlay, as clicking Drive does

  const t0 = performance.now();
  let n = 0, maxGap = 0, last = t0, sum = 0;
  await new Promise((res) => {
    function tick(now) {
      n++;
      const gap = now - last;
      if (n > 1) { maxGap = Math.max(maxGap, gap); sum += gap; }
      last = now;
      if (now - t0 < 10000) requestAnimationFrame(tick);
      else res();
    }
    requestAnimationFrame(tick);
  });
  const elapsed = performance.now() - t0;
  const fd = window.__fd;
  return JSON.stringify({
    status: status ? status.textContent.slice(0, 60) : null,
    gpuNote: gpuNote && gpuNote.style.display !== 'none' ? gpuNote.textContent.slice(0, 90) : null,
    quality: document.getElementById('quality').textContent,
    hasApp: typeof fd,
    frames: n,
    elapsedS: +(elapsed / 1000).toFixed(2),
    avgFps: +(n / (elapsed / 1000)).toFixed(1),
    avgFrameMs: n > 1 ? +(sum / (n - 1)).toFixed(1) : null,
    worstFrameMs: +maxGap.toFixed(1),
    kmh: fd ? +fd.sim.kmh.toFixed(1) : null,
    station: fd ? +fd.sim.s.toFixed(1) : null,
    errors,
  }, null, 1);
})()
