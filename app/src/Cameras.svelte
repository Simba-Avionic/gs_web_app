<script>
    // @ts-nocheck
    import { onMount } from "svelte";
    import SVG from "../public/images/rocket.svg";

    export let host;
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<style>
    .cameras {
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
    }
  
    .camera-container {
      display: grid;
      grid-template-columns: repeat(2, 340px);
      grid-template-rows: repeat(2, 220px);
      gap: 40px;
    }
  
    .camera-item {
      border: 1px solid rgba(204, 204, 220, 0.5);
      display: flex;
      justify-content: center;
      align-items: center;
      font-size: 1.5em;
      border-radius: 1vw;
    }
  </style>
  
  <div class="cameras">
    <div class="camera-container">
      <div class="camera-item">1</div>
      <div class="camera-item">2</div>
      <div class="camera-item">3</div>
      <div class="camera-item">4</div>
    </div>
  </div>
