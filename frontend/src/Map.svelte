<!-- MapWidget.svelte -->
<script>
    import { onMount, onDestroy } from "svelte";
    import TelemetryField from "./lib/TelemetryField.svelte";
    // @ts-ignore
    import L from "leaflet";

    export let host;
    let topics = [];

    let map;
    let marker;
    let path = [];
    let pathLine;
    let showPath = true;

    let gpsSocket;
    let maxAltitudeSocket;
    let gpsData;
    let maxAltitudeData;

    const customIcon = L.icon({
        iconUrl: "icons/simba_logo.png",
        iconSize: [38, 38],
        iconAnchor: [19, 38],
        popupAnchor: [0, -38],
    });

    const loadPathFromLocalStorage = () => {
        const savedPath = localStorage.getItem("markerPath");
        return savedPath ? JSON.parse(savedPath) : [];
    };

    const savePathToLocalStorage = () => {
        localStorage.setItem("markerPath", JSON.stringify(path));
    };

    onMount(() => {
        fetchConfig();
        map = L.map("map", {
            zoomControl: false,
            attributionControl: false,
        }).setView([54.4034, 18.5166], 13);
        const layer = L.tileLayer(
            `http://${host}:8000/tiles/tiles_tricity/{z}/{x}/{y}.png`,
            {
                minZoom: 10,
                maxZoom: 15,
                tileSize: 256,
                tms: false,
            },
        );

        L.control.zoom({ position: "bottomright" }).addTo(map);

        map.addLayer(layer);
        marker = L.marker([54.4034, 18.5166], { icon: customIcon }).addTo(map);

        path = loadPathFromLocalStorage();
        pathLine = L.polyline(path, { color: "orange" }).addTo(map);

        gpsSocket = new WebSocket(`ws://${host}:8000/mavlink/simba_gps`);
        gpsSocket.onmessage = (event) => {
            try {
                gpsData = JSON.parse(event.data);
                if (
                    gpsData !== "None" &&
                    gpsData !== null &&
                    gpsData.lat &&
                    gpsData.lon
                ) {
                    const newLatLng = [gpsData.lat, gpsData.lon];
                    marker.setLatLng(newLatLng);

                    path.push(newLatLng);
                    pathLine.setLatLngs(path);
                    savePathToLocalStorage();
                }
            } catch (e) {
                console.error("Error processing GPS data:", e);
            }
        };

        maxAltitudeSocket = new WebSocket(`ws://${host}:8000/mavlink/simba_max_altitude`);
        maxAltitudeSocket.onmessage = (event) => {
            try {
                maxAltitudeData = JSON.parse(event.data);
            } catch (e) {
                console.error("Error processing max altitude data:", e);
            }
        };

        map.invalidateSize();
    });

    onDestroy(() => {
        if (gpsSocket) {
            gpsSocket.close();
        }
        if (maxAltitudeSocket) {
            maxAltitudeSocket.close();
        }
    });

    const togglePath = () => {
        showPath = !showPath;
        if (showPath) {
            pathLine.addTo(map);
        } else {
            pathLine.removeFrom(map);
        }
    };

    const clearPath = () => {
        path = [];
        pathLine.setLatLngs([]);
        localStorage.removeItem("markerPath");
    };

    async function fetchConfig() {
        try {
            const response = await fetch(`http://${host}:8000/config`);
            const data = await response.json();
            
            // Filter topics to only include GPS and max altitude topics
            const allowedTopics = [
                "mavlink/simba_max_altitude",
                "mavlink/simba_altitude_orientation",
                "mavlink/simba_gps"
            ];
            
            topics = data.topics.filter(topic => 
                allowedTopics.includes(topic.topic_name || topic.name));
                
            topics = topics.map(topic => {
                return {
                    id: topic.id || Math.random().toString(),
                    topic_name: topic.topic_name || topic.name,
                    msg_fields: topic.msg_fields || []
                };
            });
            
            console.log("Filtered topics:", topics);
        } catch (error) {
            console.error("Error fetching config:", error);
        }
    }
</script>

<div id="map">
    <div class="info-widget">
        <div class="buttons">
            <button class="button" on:click={togglePath}>
                {showPath ? "Hide Path" : "Show Path"}
            </button>
            <button class="button" on:click={clearPath}>Clear Path</button>
        </div>
        <div class="fields-container">
            {#each topics as topic (topic.id)}
                <TelemetryField {topic} {host} />
            {/each}
        </div>
    </div>
</div>

<style>
    #map {
        height: 100vh;
        width: 100%;
        position: relative;
    }

    .buttons {
        display: flex;
        align-items: center;
        gap: 25px;
        justify-content: center;
    }

    .button {
        padding: 10px 16px;
        border: none;
        border-radius: 0.2rem;
        cursor: pointer;
        display: inline-flex;
        align-items: center;
        font-weight: 500;
        background: rgb(61, 113, 217);
        width: 160px;
        justify-content: center;
        text-align: center;
        font-size: 1rem;
    }

    .button:hover {
        background-color: rgb(90, 134, 222);
        transition:
            background-color 0.3s,
            color 0.3s;
    }

    .fields-container {
        display: flex;
        flex-direction: column;
        gap: 8px;
        margin-top: 16px;
        max-height: calc(100% - 60px);
        overflow-y: auto;
    }

    .info-widget {
        position: absolute;
        top: calc(var(--navbar-height) + 20px);
        left: 75%;
        width: 20%;
        height: 40%;
        padding: 16px;
        border: 1px solid var(--border-color);
        border-radius: 0.75rem;
        background-color: var(--bg-color);
        color: inherit;
        z-index: 1000;
        display: flex;
        flex-direction: column;
        justify-content: flex-start;
        overflow-y: auto;
        line-height: 1.5rem;
        font-size: 0.9rem;
        box-shadow: rgba(0, 0, 0, 0.24) 0px 3px 8px;
        width: 350px;
        left: auto;
        right: 20px;
    }

</style>
