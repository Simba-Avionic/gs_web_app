<script>
  import CameraView from "./lib/CameraView.svelte";
  export let host;
</script>

<div class="main-container">
  <div class="videos">
    <CameraView host={host} camera="camera" hasPTZ={true} />
    <CameraView host={host} camera="camera2" hasPTZ={false} />
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

  .videos {
    display: flex;
    flex-direction: column;
    gap: 10px;
    flex: 1;
    height: 100%;
  }

  @media (max-width: 2560px) {
    video { max-width: 1280px; }
  }
  @media (max-width: 1920px) {
    video { max-width: 960px; }
  }
  @media (max-width: 1280px) {
    video { max-width: 640px; }
  }
  @media (max-width: 1024px) {
    video { max-width: 512px; }
  }
</style>
