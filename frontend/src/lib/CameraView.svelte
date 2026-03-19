<script>
  import Hls from "hls.js";
  import { onMount, onDestroy } from "svelte";

  export let camera; // e.g. "camera1" or "camera2"
  export let hasPTZ = false;

  let host;
  let containerEl; // new: wrapper element to observe
  let videoEl;
  let hls;
  let streamUnavailable = false;
  let probeAbort = null;
  let setupInProgress = false;
  let io = null; // IntersectionObserver

  // PTZ state
  let ptzInterval = null;
  let isRecording = false;

  let showControls = false;

  async function probeStream(url, timeoutMs = 2500) {
    const controller = new AbortController();
    probeAbort = controller;
    const timeout = setTimeout(() => controller.abort(), timeoutMs);

    try {
      const res = await fetch(url, {
        method: "GET",
        cache: "no-cache",
        signal: controller.signal,
      });
      clearTimeout(timeout);
      probeAbort = null;
      return res && res.ok;
    } catch (err) {
      clearTimeout(timeout);
      probeAbort = null;
      return false;
    }
  }

  async function doSetupVideo() {
    if (setupInProgress) return;
    setupInProgress = true;

    const streamUrl = `http://${host}/camera/${camera}/stream.m3u8`;

    try {
      // 1. Poll the backend until we get a 200 OK (not a 202)
      let ready = false;
      let attempts = 0;

      while (!ready && attempts <= 5) {
        const res = await fetch(streamUrl, {
          method: "GET",
          cache: "no-cache",
        });
        if (res.status === 200) {
          ready = true;
        } else {
          console.log(`Stream preparing for ${camera}, attempt: ${attempts}`);
          await new Promise((r) => setTimeout(r, 1000)); // Wait 1 second
          attempts++;
        }
      }

      if (!ready) {
        streamUnavailable = true;
        return;
      }

      streamUnavailable = false;
      if (Hls.isSupported()) {
        if (hls) hls.destroy();

        hls = new Hls({ liveSyncDuration: 1, liveMaxLatencyDuration: 3 });

        hls.on(Hls.Events.ERROR, (event, data) => {
          if (data.fatal) {
            console.warn("Fatal HLS error, retrying setup...");
            teardownVideo();
            setTimeout(doSetupVideo, 2000);
          }
        });

        hls.loadSource(streamUrl);
        hls.attachMedia(videoEl);
        videoEl.play().catch(() => {});
      }
    } catch (err) {
      console.error("Setup error:", err);
      streamUnavailable = true;
    } finally {
      setupInProgress = false;
    }
  }

  function teardownVideo() {
    // Abort probe if running
    if (probeAbort) {
      try {
        probeAbort.abort();
      } catch (err) {}
      probeAbort = null;
    }
    // Destroy hls if attached
    if (hls) {
      try {
        hls.destroy();
      } catch (err) {}
      hls = null;
    }
    // If a video src was set (native), clear it
    if (videoEl) {
      try {
        videoEl.pause();
        videoEl.removeAttribute("src");
        videoEl.load && videoEl.load();
      } catch (err) {}
    }
  }

  onMount(() => {
    // Setup IntersectionObserver to only probe/attach when visible in DOM
    host = window.location.host;
    try {
      io = new IntersectionObserver(
        (entries) => {
          entries.forEach((entry) => {
            if (entry.isIntersecting && entry.intersectionRatio > 0) {
              // element became visible -> attempt to setup
              doSetupVideo().catch((e) =>
                console.warn("doSetupVideo error:", e),
              );
            } else {
              // element not visible -> teardown to avoid background activity/errors
              teardownVideo();
            }
          });
        },
        { root: null, threshold: 0.01 },
      );

      if (containerEl) io.observe(containerEl);
    } catch (err) {
      // IntersectionObserver not available or failed — fall back to immediate setup
      console.debug(
        "IntersectionObserver unavailable; falling back to immediate camera setup",
        err,
      );
      doSetupVideo().catch((e) => console.warn("doSetupVideo error:", e));
    }
  });

  onDestroy(() => {
    // disconnect observer
    try {
      if (io && containerEl) {
        io.unobserve(containerEl);
        io.disconnect();
        io = null;
      }
    } catch (err) {}

    teardownVideo();
    stopPTZ();
  });

  // ---------------- PTZ ----------------
  async function sendPTZ(pan, tilt, zoom = 0, speed = 10) {
    try {
      await fetch(`http://${host}/ptz/${camera}/move`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ pan, tilt, zoom, speed }),
      });
    } catch (err) {
      console.warn("PTZ request failed:", err);
    }
  }

  function startPTZ(pan, tilt) {
    stopPTZ();
    ptzInterval = setInterval(() => sendPTZ(pan, tilt), 100);
  }

  function stopPTZ() {
    if (ptzInterval) {
      clearInterval(ptzInterval);
      ptzInterval = null;
    }
    sendPTZ(0, 0, 0, 0).catch(() => {});
  }

  // ---------------- Recording ----------------
  async function startRecording() {
    try {
      const res = await fetch(
        `http://${host}/camera/${camera}/start_recording`,
        {
          method: "POST",
        },
      );
      const data = await res.json();
      if (data.status === "recording started") isRecording = true;
    } catch (err) {
      console.warn("startRecording failed:", err);
    }
  }

  async function stopRecording() {
    try {
      const res = await fetch(
        `http://${host}/camera/${camera}/stop_recording`,
        {
          method: "POST",
        },
      );
      const data = await res.json();

      if (data.status === "recording stopped") {
        isRecording = false;

        const downloadUrl = `http://${host}/camera/${camera}/download_recording`;
        const checkRes = await fetch(downloadUrl, { method: "HEAD" });

        if (checkRes.ok) {
          const link = document.createElement("a");
          link.href = downloadUrl;
          link.download = `${camera}_recording.mp4`;
          document.body.appendChild(link);
          link.click();
          document.body.removeChild(link);
        } else {
          console.warn("Plik jeszcze nie jest gotowy na dysku.");
        }
      }
    } catch (err) {
      console.error("Błąd zatrzymywania:", err);
    }
  }
</script>

<div class="video-container" bind:this={containerEl}>
  <div class="video-wrapper" class:is-recording={isRecording}>
    {#if streamUnavailable}
      <div class="stream-placeholder">
        <div>Camera <em>{camera}</em> is not available</div>
        <div class="hint">Stream file not found or server not responding</div>
      </div>
    {:else}
      <video bind:this={videoEl} autoplay muted playsinline controls></video>
    {/if}
  </div>

  <div class="controls-toggle">
    <button class="toggle-btn" on:click={() => (showControls = !showControls)}>
      {showControls ? "▲ Hide Controls" : "▼ Show Controls"}
    </button>
  </div>

  {#if showControls}
    <div class="controls dropdown">
      <div class="ptz-record-wrapper">
        {#if hasPTZ}
          <div class="ptz-controls">
            <div class="row">
              <button
                on:mousedown={() => startPTZ(0, 20)}
                on:mouseup={stopPTZ}
                on:mouseleave={stopPTZ}
                on:touchstart={() => startPTZ(0, 20)}
                on:touchend={stopPTZ}>▲</button
              >
            </div>
            <div class="row">
              <button
                on:mousedown={() => startPTZ(-20, 0)}
                on:mouseup={stopPTZ}
                on:mouseleave={stopPTZ}
                on:touchstart={() => startPTZ(-20, 0)}
                on:touchend={stopPTZ}>◀</button
              >
              <button on:click={stopPTZ}>■</button>
              <button
                on:mousedown={() => startPTZ(20, 0)}
                on:mouseup={stopPTZ}
                on:mouseleave={stopPTZ}
                on:touchstart={() => startPTZ(20, 0)}
                on:touchend={stopPTZ}>▶</button
              >
            </div>
            <div class="row">
              <button
                on:mousedown={() => startPTZ(0, -20)}
                on:mouseup={stopPTZ}
                on:mouseleave={stopPTZ}
                on:touchstart={() => startPTZ(0, -20)}
                on:touchend={stopPTZ}>▼</button
              >
            </div>
          </div>
        {:else}
          <div class="ptz-controls">
            <div class="row">
              <button on:click={() => sendPTZ(0, 20)}>▲</button>
            </div>
            <div class="row">
              <button on:click={() => sendPTZ(-20, 0)}>◀</button>
              <button on:click={() => sendPTZ(0, 0)}>■</button>
              <button on:click={() => sendPTZ(20, 0)}>▶</button>
            </div>
            <div class="row">
              <button on:click={() => sendPTZ(0, -20)}>▼</button>
            </div>
          </div>
        {/if}

        <div class="record-controls">
          <button
            class="record-btn"
            title="Start Recording"
            on:click={startRecording}>Record</button
          >
          <button
            class="stop-btn"
            title="Stop Recording"
            on:click={stopRecording}>Stop</button
          >
        </div>
      </div>
    </div>
  {/if}
</div>

<style>
  .video-container {
    display: flex;
    flex-direction: column; /* Stack video and controls vertically */
    align-items: center;
    width: 100%;
    flex: 1;
  }

  /* Force 16:9 ratio and handle bounds */
  .video-wrapper {
    width: 100%;
    aspect-ratio: 16 / 9;
    background: var(--snd-bg-color);
    border-radius: 1rem;
    display: flex;
    justify-content: center;
    align-items: center;
    overflow: hidden;
    border: 3px solid transparent;
    box-sizing: border-box;
    transition:
      border-color 0.2s ease,
      box-shadow 0.2s ease;
  }

  .video-wrapper.is-recording {
    border-color: #c41934;
    animation: recordPulse 2s ease-in-out infinite;
  }

  video {
    width: 100%;
    height: 100%;
    object-fit: contain;
    /* No border needed here anymore */
  }

  .stream-placeholder {
    width: 100%;
    height: 100%;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    color: var(--text-color);
    text-align: center;
    padding: 1rem;
    box-sizing: border-box;
  }

  .stream-placeholder .hint {
    margin-top: 0.5rem;
    font-size: 0.85rem;
    color: var(--selection-color);
  }

  /* Dropdown UI styling */
  .controls-toggle {
    width: 100%;
    display: flex;
    justify-content: center;
    margin-top: 20px;
  }

  .toggle-btn {
    font-size: 0.9rem;
    padding: 0.4em 1em;
    border-radius: 4px;
    background-color: var(--snd-bg-color, #333);
    color: var(--text-color, #fff);
  }

  .controls.dropdown {
    margin-top: 16px;
    padding: 20px;
    width: 100%;
    max-width: 400px;
    background-color: var(--snd-bg-color, #222);
    border: 1px solid var(--border-color, #444);
    border-radius: 8px;
    display: flex;
    justify-content: center;
    animation: slideDown 0.2s ease-out forwards;
  }

  @keyframes slideDown {
    from {
      opacity: 0;
      transform: translateY(-5px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }

  @keyframes recordPulse {
    0% {
      box-shadow: 0 0 8px rgba(196, 25, 52, 0.6), 
                  0 0 16px rgba(222, 48, 77, 0.3);
    }
    50% {
      box-shadow: 0 0 16px rgba(196, 25, 52, 1), 
                  0 0 32px rgba(222, 48, 77, 0.6);
    }
    100% {
      box-shadow: 0 0 8px rgba(196, 25, 52, 0.6), 
                  0 0 16px rgba(222, 48, 77, 0.3);
    }
  }

  .ptz-record-wrapper {
    display: flex;
    gap: 48px;
    align-items: center;
  }

  .ptz-controls {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }

  .row {
    display: flex;
    gap: 8px;
    justify-content: center;
  }

  button {
    padding: 0.4em 0.8em;
    font-size: 1.2em;
    cursor: pointer;
    border-radius: 4px;
    border: 1px solid var(--border-color, #555);
    background-color: var(--bg-color, #111);
    color: var(--text-color, #fff);
    transition: background-color 0.2s ease;
  }

  button:hover {
    background-color: var(--nav-hover, #444);
  }

  .record-controls {
    display: flex;
    flex-direction: column;
    gap: 10px;
    justify-content: center;
  }

  .record-btn {
    font-size: 1rem;
    background-color: #c41934;
    border-color: #a0142a;
  }
  .record-btn:hover {
    background-color: #a0142a;
  }

  .stop-btn {
    font-size: 1rem;
  }
</style>
