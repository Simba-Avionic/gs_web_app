<script>
  import CameraView from "./lib/CameraView.svelte";
</script>

<div class="main-container">
  <div class="videos">
    <CameraView camera="camera1" hasPTZ={true} />
    <CameraView camera="camera2" hasPTZ={false} />
  </div>
</div>

<style>
  .main-container {
    display: flex;
    gap: 30px; /* Increased from 20px */
    margin: 0 auto;
    margin-top: var(--navbar-height);
    height: calc(100vh - var(--navbar-height));
    padding: 2rem; /* Increased from 1rem */
    max-width: 1800px; /* Keeps it framed nicely on huge screens */
    box-sizing: border-box;
    overflow: hidden;
  }

  .videos {
    display: flex;
    flex-direction: row;
    gap: 40px; /* Increased from 20px for clear separation between cameras */
    flex: 1;
    width: 100%;
    height: 100%;
  }

  /* .videos > * {
    flex: 1;
    min-width: 0;
    min-height: 0;
  } */
</style>