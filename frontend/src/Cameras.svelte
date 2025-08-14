<script>
  import Hls from 'hls.js';
  import { onMount, onDestroy } from 'svelte';

  export let host;

  let videoEl;
  let hls;

  onMount(() => {
    const streamUrl = `http://${host}:8000/camera/stream.m3u8`;

    if (Hls.isSupported()) {
      hls = new Hls({
        liveSyncDuration: 1,
        liveMaxLatencyDuration: 3
      });
      hls.loadSource(streamUrl);
      hls.attachMedia(videoEl);
    } else if (videoEl.canPlayType('application/vnd.apple.mpegurl')) {
      videoEl.src = streamUrl;
    }

    videoEl.play().catch(err => console.error("Autoplay failed:", err));
  });

  onDestroy(() => {
    if (hls) {
      hls.destroy();
      hls = null;
    }
  });

  async function sendPTZ(pan, tilt, zoom = 0, speed = 0.5) {
    try {
      const response = await fetch(`http://${host}:8000/ptz/move`, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({ pan, tilt, zoom, speed })
      });
      if (!response.ok) {
        const err = await response.json();
        alert("PTZ Error: " + err.detail);
      }
    } catch(e) {
      alert("Network error sending PTZ command.");
      console.error(e);
    }
  }

  function moveLeft() { sendPTZ(-0.5, 0); }
  function moveRight() { sendPTZ(0.5, 0); }
  function moveUp() { sendPTZ(0, 0.5); }
  function moveDown() { sendPTZ(0, -0.5); }
  function stop() { sendPTZ(0, 0, 0, 0); }
</script>

<style>
  .container {
    width: 100%;
    max-width: 640px; /* max width of video player */
    margin: auto;
  }

  video {
    width: 100%;
    height: 360px;  /* fixed height, not fullscreen */
    background: black;
    border-radius: 4px;
    object-fit: contain;
  }

  .ptz-controls {
    margin-top: 10px;
    display: flex;
    flex-direction: column;
    align-items: center;
    user-select: none;
  }

  .row {
    display: flex;
    gap: 10px;
    margin: 5px 0;
  }

  button {
    padding: 0.5em 1em;
    font-size: 1.1em;
    cursor: pointer;
    border-radius: 4px;
    border: 1px solid #444;
    background-color: #eee;
    transition: background-color 0.2s ease;
  }
  button:hover {
    background-color: #ddd;
  }
</style>

<div class="container">
  <video bind:this={videoEl} autoplay muted playsinline controls></video>

  <div class="ptz-controls">
    <div class="row">
      <button on:click={moveUp}>⬆️ Up</button>
    </div>
    <div class="row">
      <button on:click={moveLeft}>⬅️ Left</button>
      <button on:click={stop}>⏹ Stop</button>
      <button on:click={moveRight}>➡️ Right</button>
    </div>
    <div class="row">
      <button on:click={moveDown}>⬇️ Down</button>
    </div>
  </div>
</div>
