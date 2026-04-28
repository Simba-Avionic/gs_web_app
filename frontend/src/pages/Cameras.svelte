<script>
  import CameraView from "../components/common/CameraView.svelte";
</script>

<div class="main-container">
  <div class="videos">
    <CameraView camera="camera1" hasPTZ={true} />
    <CameraView camera="camera2" hasPTZ={true} />
  </div>
</div>

<style>
  .main-container {
    display: flex;
    gap: 30px;
    margin: 0 auto;
    margin-top: var(--navbar-height);
    height: calc(100vh - var(--navbar-height));
    padding: 2rem;
    max-width: 1800px;
    box-sizing: border-box;
    overflow: hidden;
  }

  .videos {
    display: flex;
    flex-direction: row;
    gap: 40px;
    flex: 1;
    width: 100%;
    height: 100%;
  }
</style>