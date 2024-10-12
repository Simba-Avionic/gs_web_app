<!-- MapWidget.svelte -->
<script>
    import { onMount, onDestroy } from 'svelte';
    import L from 'leaflet';
 
    let map;
 
    onMount(() => {
        const host = process.env.IP_ADDRESS;
        // 54.351887748, 18.646338873.
        map = L.map('map').setView([54.351887748, 18.646338873], 8);
        L.tileLayer(`http://${host}:8000/tiles/tiles_tricity/{z}/{x}/{y}.png`, {
            maxZoom: 13,
            minZoom: 10,
            tileSize: 256
        }).addTo(map);
    });
 
    onDestroy(() => {
        map.remove();
    });
 </script>
 
 <style>

    .map-container {
        position: absolute;
        top: 15vh;
        left: 67vw;
        width: 30vw;
        height: 75vh;
        border-radius: 1vw; /* Adjust the border-radius for a rounded rectangle */
        overflow: hidden;
        border: 1px solid #eee; /* Optional: Add a border */
   }

    #map {
       height: 100%;
       width: 100%;
    }

 </style>
 
 <div class="map-container">
    <div id="map"></div>
 </div>
 