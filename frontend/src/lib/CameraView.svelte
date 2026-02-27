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

      while (!ready && attempts < 10) {
        const res = await fetch(streamUrl, {
          method: "GET",
          cache: "no-cache",
        });
        if (res.status === 200) {
          ready = true;
        } else {
          console.log("Stream preparing... waiting 1s");
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
  {#if streamUnavailable}
    <div class="stream-placeholder">
      <div>Camera {camera} is not available</div>
      <div class="hint">Stream file not found or server not responding</div>
    </div>
  {:else}
    <video
      bind:this={videoEl}
      autoplay
      muted
      playsinline
      controls
      class:is-recording={isRecording}
    ></video>
  {/if}

  <div class="controls">
    <div class="ptz-record-wrapper">
      <!-- PTZ controls -->
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

      <!-- Recording buttons (vertical) -->
      <div class="record-controls">
        <button
          class="record-btn"
          title="Start Recording"
          on:click={startRecording}>Record</button
        >
        <button class="stop-btn" title="Stop Recording" on:click={stopRecording}
          >Stop</button
        >
      </div>
    </div>
  </div>
</div>

<style>
  .video-container {
    display: flex;
    gap: 10px;
    justify-content: space-evenly;
    align-items: stretch;
    flex: 1;
  }

  video {
    flex: 1;
    width: 100%;
    height: 100%;
    max-width: 640px;
    background: black;
    border-radius: 5px;
    object-fit: contain;
    border: 3px solid transparent;
    transition: border-color 0.2s ease;
  }

  video.is-recording {
    border-color: #c41934;
  }

  .stream-placeholder {
    flex: 1;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    background: #111;
    color: #eee;
    border-radius: 5px;
    padding: 1rem;
    text-align: center;
  }

  .stream-placeholder .hint {
    margin-top: 0.5rem;
    font-size: 0.85rem;
    color: #bbb;
  }

  .controls {
    display: flex;
    justify-content: center;
    align-items: center;
  }

  .ptz-record-wrapper {
    display: flex;
    gap: 50px;
    align-items: center;
  }

  .ptz-controls {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .row {
    display: flex;
    gap: 10px;
    justify-content: center;
  }

  button {
    padding: 0.5em 1em;
    font-size: 1.4em;
    cursor: pointer;
    border-radius: 4px;
    border: 1px solid var(--border-color);
    background-color: var(--bg-color);
    transition: background-color 0.2s ease;
  }

  button:hover {
    background-color: var(--nav-hover);
  }

  .record-controls {
    display: flex;
    flex-direction: column;
    gap: 10px;
    justify-content: center;
  }

  @media (max-width: 2560px) {
    video {
      max-width: 960px;
    }
  }
  @media (max-width: 1920px) {
    video {
      max-width: 640px;
    }
  }
  @media (max-width: 1280px) {
    video {
      max-width: 480px;
    }
  }
  @media (max-width: 1024px) {
    video {
      max-width: 512px;
    }
  }
</style>
