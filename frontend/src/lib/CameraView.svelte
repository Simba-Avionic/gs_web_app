<script>
  import Hls from "hls.js";
  import { onMount, onDestroy } from "svelte";

  export let host;
  export let camera; // e.g. "camera" or "camera2"
  export let hasPTZ = false;

  let videoEl;
  let hls;

  // PTZ state
  let ptzInterval = null;
  let isRecording = false;

  function setupVideo() {
    const streamUrl = `http://${host}/camera/${camera}/stream.m3u8`;

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
    await fetch(`http://${host}/ptz/${camera}/move`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ pan, tilt, zoom, speed }),
    });
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
  async function startRecording() {
    const res = await fetch(`http://${host}/camera/${camera}/start_recording`, {
      method: "POST",
    });
    const data = await res.json();
    if (data.status === "recording started") isRecording = true;
  }

  async function stopRecording() {
    const res = await fetch(`http://${host}/camera/${camera}/stop_recording`, {
      method: "POST",
    });
    const data = await res.json();
    if (data.status === "recording stopped") {
      isRecording = false;
      // Auto-download
      window.location.href = `http://${host}/camera/${camera}/download_recording`;
    }
  }
</script>

<div class="video-container">
  <video
    bind:this={videoEl}
    autoplay
    muted
    playsinline
    controls
    class:is-recording={isRecording}
  ></video>

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
              on:touchend={stopPTZ}>▲</button
            >
          </div>
          <div class="row">
            <button
              on:mousedown={() => startPTZ(-10, 0)}
              on:mouseup={stopPTZ}
              on:mouseleave={stopPTZ}
              on:touchstart={() => startPTZ(-10, 0)}
              on:touchend={stopPTZ}>◀</button
            >
            <button on:click={stopPTZ}>■</button>
            <button
              on:mousedown={() => startPTZ(10, 0)}
              on:mouseup={stopPTZ}
              on:mouseleave={stopPTZ}
              on:touchstart={() => startPTZ(10, 0)}
              on:touchend={stopPTZ}>▶</button
            >
          </div>
          <div class="row">
            <button
              on:mousedown={() => startPTZ(0, -10)}
              on:mouseup={stopPTZ}
              on:mouseleave={stopPTZ}
              on:touchstart={() => startPTZ(0, -10)}
              on:touchend={stopPTZ}>▼</button
            >
          </div>
        </div>
      {:else}
        <div class="ptz-controls">
          <div class="row">
            <button on:click={() => sendPTZ(0, 10)}>▲</button>
          </div>
          <div class="row">
            <button on:click={() => sendPTZ(-10, 0)}>◀</button>
            <button on:click={() => sendPTZ(0, 0)}>■</button>
            <button on:click={() => sendPTZ(10, 0)}>▶</button>
          </div>
          <div class="row">
            <button on:click={() => sendPTZ(0, -10)}>▼</button>
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
