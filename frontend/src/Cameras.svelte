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

  .videos > * {
  flex: 1;          /* take equal available space */
  min-height: 0;    /* prevent flexbox from overflowing */
}
</style>
