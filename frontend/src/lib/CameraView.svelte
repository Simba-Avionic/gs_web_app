<script>
  import Hls from "hls.js";
  import { onMount, onDestroy } from "svelte";

  export let host;
  export let camera;        // e.g. "camera" or "camera2"
  export let hasPTZ = false;

  let videoEl;
  let hls;

  // PTZ state
  let ptzInterval = null;

  // Recording state
  let mediaRecorder;
  let recordedChunks = [];
  let isRecording = false;

  function setupVideo() {
    const streamUrl = `http://${host}/${camera}/stream.m3u8`;

    if (Hls.isSupported()) {
      hls = new Hls({ liveSyncDuration: 1, liveMaxLatencyDuration: 3 });
      hls.loadSource(streamUrl);
      hls.attachMedia(videoEl);
      videoEl.play().catch((err) => console.error("Autoplay failed:", err));
    } else if (videoEl.canPlayType("application/vnd.apple.mpegurl")) {
      videoEl.src = streamUrl;
      videoEl.play().catch((err) => console.error("Autoplay failed:", err));
    }
  }

  onMount(setupVideo);

  onDestroy(() => {
    if (hls) {
      hls.destroy();
      hls = null;
    }
    stopPTZ();
  });

  // ---------------- PTZ ----------------
  async function sendPTZ(pan, tilt, zoom = 0, speed = 10) {
    try {
      await fetch(`http://${host}/ptz/move`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ pan, tilt, zoom, speed }),
      });
    } catch (e) {
      console.error("PTZ error:", e);
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
    sendPTZ(0, 0, 0, 0);
  }

  // ---------------- Recording ----------------
  function startRecording() {
    if (isRecording) return;
    const stream = videoEl.captureStream();
    mediaRecorder = new MediaRecorder(stream, { mimeType: "video/webm; codecs=vp9" });
    recordedChunks = [];

    mediaRecorder.ondataavailable = (event) => {
      if (event.data.size > 0) recordedChunks.push(event.data);
    };

    mediaRecorder.onstop = () => {
      const blob = new Blob(recordedChunks, { type: "video/webm" });
      const url = URL.createObjectURL(blob);

      // Auto-download file
      const a = document.createElement("a");
      a.href = url;
      a.download = `${camera}_recording.webm`;
      a.click();
      URL.revokeObjectURL(url);
    };

    mediaRecorder.start();
    isRecording = true;
  }

  function stopRecording() {
    if (!isRecording) return;
    mediaRecorder.stop();
    isRecording = false;
  }
</script>

<div class="video-container">
  <video
    bind:this={videoEl}
    autoplay
    muted
    playsinline
    controls
    class:is-recording={isRecording}></video>

  <div class="controls">
    <div class="ptz-record-wrapper">
      <!-- PTZ controls -->
      {#if hasPTZ}
        <div class="ptz-controls">
          <div class="row">
            <button
              on:mousedown={() => startPTZ(0, 10)}
              on:mouseup={stopPTZ}
              on:mouseleave={stopPTZ}
              on:touchstart={() => startPTZ(0, 10)}
              on:touchend={stopPTZ}>▲</button>
          </div>
          <div class="row">
            <button
              on:mousedown={() => startPTZ(-10, 0)}
              on:mouseup={stopPTZ}
              on:mouseleave={stopPTZ}
              on:touchstart={() => startPTZ(-10, 0)}
              on:touchend={stopPTZ}>◀</button>
            <button on:click={stopPTZ}>■</button>
            <button
              on:mousedown={() => startPTZ(10, 0)}
              on:mouseup={stopPTZ}
              on:mouseleave={stopPTZ}
              on:touchstart={() => startPTZ(10, 0)}
              on:touchend={stopPTZ}>▶</button>
          </div>
          <div class="row">
            <button
              on:mousedown={() => startPTZ(0, -10)}
              on:mouseup={stopPTZ}
              on:mouseleave={stopPTZ}
              on:touchstart={() => startPTZ(0, -10)}
              on:touchend={stopPTZ}>▼</button>
          </div>
        </div>
      {:else}
        <div class="ptz-controls">
          <div class="row"><button on:click={() => sendPTZ(0, 10)}>▲</button></div>
          <div class="row">
            <button on:click={() => sendPTZ(-10, 0)}>◀</button>
            <button on:click={() => sendPTZ(0, 0)}>■</button>
            <button on:click={() => sendPTZ(10, 0)}>▶</button>
          </div>
          <div class="row"><button on:click={() => sendPTZ(0, -10)}>▼</button></div>
        </div>
      {/if}

      <!-- Recording buttons (vertical) -->
      <div class="record-controls">
        <button
          class="record-btn"
          title="Start Recording"
          on:click={startRecording}>Record 🔴</button>
        <button
          class="stop-btn"
          title="Stop Recording"
          on:click={stopRecording}>Stop ⬛</button>
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
    border-color: red;
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
</style>
