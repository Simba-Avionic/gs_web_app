<!-- MapWidget.svelte -->
<script>
    import { onMount, onDestroy } from 'svelte';
    // @ts-ignore
    import L from 'leaflet';
 
    let map;
    export let host;
 
    onMount(() => {
        // 54.351887748, 18.646338873.
        map = L.map('map', {attributionControl: false}).setView([54.4034, 18.5166], 11);
        const layer = L.tileLayer(`http://${host}:8000/tiles/tiles_tricity/{z}/{x}/{y}.png`, {
            minZoom: 10, maxZoom: 15, tileSize: 256, tms: false})

        map.addLayer(layer);
    });
 </script>
 
 <style>

    .map-container {
        position: absolute;
        top: 15vh;
        left: 67vw;
        width: 30vw;
        height: 75vh;
        border-radius: 1vw;
        overflow: hidden;
        border: 1px solid rgba(204, 204, 220, 0.5);
   }

    #map {
       height: 100%;
       width: 100%;
    }

</style>
 
<div id="map"></div>

 