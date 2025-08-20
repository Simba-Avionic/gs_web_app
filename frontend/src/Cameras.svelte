<script>
  import Hls from "hls.js";
  import { onMount, onDestroy } from "svelte";

  export let host;

  let videoEl1;
  let videoEl2;
  let hls1;
  let hls2;

  function setupVideo(videoEl, camera) {
    const streamUrl = `http://${host}/${camera}/stream.m3u8`;

    if (Hls.isSupported()) {
      const hls = new Hls({ liveSyncDuration: 1, liveMaxLatencyDuration: 3 });
      hls.loadSource(streamUrl);
      hls.attachMedia(videoEl);
      videoEl.play().catch((err) => console.error("Autoplay failed:", err));
      return hls;
    } else if (videoEl.canPlayType("application/vnd.apple.mpegurl")) {
      videoEl.src = streamUrl;
      videoEl.play().catch((err) => console.error("Autoplay failed:", err));
      return null;
    }
  }

  onMount(() => {
    hls1 = setupVideo(videoEl1, "camera1");
    hls2 = setupVideo(videoEl2, "camera2");
  });

  onDestroy(() => {
    if (hls1) {
      hls1.destroy();
      hls1 = null;
    }
    if (hls2) {
      hls2.destroy();
      hls2 = null;
    }
  });

  async function sendPTZ(pan, tilt, zoom = 0, speed = 0.5) {
    try {
      const response = await fetch(`http://${host}/ptz/move`, {
        // controls first camera
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ pan, tilt, zoom, speed }),
      });
      if (!response.ok) {
        const err = await response.json();
        // alert("PTZ Error: " + err.detail);
      }
    } catch (e) {
      console.log("Network error sending PTZ command.");
      console.error(e);
    }
  }

  function moveLeft() {
    sendPTZ(-0.5, 0);
  }
  function moveRight() {
    sendPTZ(0.5, 0);
  }
  function moveUp() {
    sendPTZ(0, 0.5);
  }
  function moveDown() {
    sendPTZ(0, -0.5);
  }
  function stop() {
    sendPTZ(0, 0, 0, 0);
  }
</script>

<div class="main-container">
  <div class="videos">
    <div class="video-container">
      <video bind:this={videoEl1} autoplay muted playsinline controls></video>
      <div class="ptz-controls">
        <div class="row">
          <button on:click={moveUp}>🠉</button>
        </div>
        <div class="row">
          <button on:click={moveLeft}>🠈</button>
          <button on:click={stop}>⏹</button>
          <button on:click={moveRight}>🠊</button>
        </div>
        <div class="row">
          <button on:click={moveDown}>🠋</button>
        </div>
      </div>
    </div>
    <div class="video-container">
      <video bind:this={videoEl2} autoplay muted playsinline controls></video>
      <div class="ptz-controls">
        <div class="row">
          <button on:click={moveUp}>🠉</button>
        </div>
        <div class="row">
          <button on:click={moveLeft}>🠈</button>
          <button on:click={stop}>⏹</button>
          <button on:click={moveRight}>🠊</button>
        </div>
        <div class="row">
          <button on:click={moveDown}>🠋</button>
        </div>
      </div>
    </div>
  </div>
</div>

<style>
  .main-container {
    display: flex;
    gap: 20px;
    margin: 0 auto;
    margin-top: var(--navbar-height);
    height: calc(100vh - var(--navbar-height));
    padding: 1rem;
    box-sizing: border-box;
    overflow: hidden;
  }

  .video-container {
    display: flex; /* horizontal layout */
    gap: 10px; /* space between video and controls */
    justify-content: space-evenly; /* space out video and controls */
    align-items: stretch; /* make children fill height */
    flex: 1;
  }

  .videos {
    display: flex;
    flex-direction: column;
    gap: 10px;
    flex: 1;
    height: 100%;
  }

  video {
    flex: 1;
    width: 100%;
    height: 100%;
    max-width: 640px;
    background: black;
    border-radius: 5px;
    object-fit: contain;
  }

  .ptz-controls {
    display: flex;
    flex-direction: column;
    justify-content: center;
    gap: 10px;
  }

  .row {
    display: flex;
    gap: 10px;
    justify-content: center;
  }

  button {
    padding: 0.5em 1em;
    font-size: 1.5em;
    cursor: pointer;
    border-radius: 4px;
    border: 1px solid var(--border-color);
    background-color: var(--bg-color);
    transition: background-color 0.2s ease;
  }

  button:hover {
    background-color: var(--nav-hover);
  }

  @media (max-width: 2560px) {
    video {
      max-width: 1280px;
    }
  }

  @media (max-width: 1920px) {
    video {
      max-width: 960px;
    }
  }

  @media (max-width: 1280px) {
    video {
      max-width: 640px;
    }
  }

  @media (max-width: 1024px) {
    video {
      max-width: 512px;
    }
  }
</style>
