<script>
    // @ts-nocheck
    import { onMount } from "svelte";
    import MapWidget from './MapWidget.svelte';
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<div class='map-container'>
    <MapWidget />
</div>


<style>
    .map-container {
        position: absolute;
        top: calc(50% + 20px);
        left: 50%;
        transform: translate(-50%, -50%); /* Center it */
        width: 70vw;
        height: 70vh;
        border-radius: 2vw; /* Adjust the border-radius for a rounded rectangle */
        overflow: hidden;
        border: 1px solid #eee; /* Optional: Add a border */
    }

</style>