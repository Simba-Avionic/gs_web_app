<script>
    // @ts-nocheck
    import { onMount } from "svelte";
    import SVG from "../public/images/rocket.svg";
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<style>
    .container {
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
    }
  
    .grid-container {
      display: grid;
      grid-template-columns: repeat(2, 300px);
      grid-template-rows: repeat(2, 200px);
      gap: 40px;
    }
  
    .grid-item {
      border: 1px solid #eee;
      display: flex;
      justify-content: center;
      align-items: center;
      font-size: 1.5em;
      border-radius: 1vw;
    }
  </style>
  
  <div class="container">
    <div class="grid-container">
      <div class="grid-item">1</div>
      <div class="grid-item">2</div>
      <div class="grid-item">3</div>
      <div class="grid-item">4</div>
    </div>
  </div>
