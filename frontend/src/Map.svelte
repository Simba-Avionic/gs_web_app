<!-- MapWidget.svelte -->
<script>
    import { onMount, onDestroy } from "svelte";
    import { rosTimeToFormattedTime } from "./lib/Utils.svelte";
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
    let socket;
    let data;

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

        socket = new WebSocket(`ws://${host}:8000/rocket/telemetry`);
        socket.onmessage = (event) => {
            data = JSON.parse(event.data);
            if (data.latitude && data.longitude) {
                const newLatLng = [data.latitude, data.longitude];
                marker.setLatLng(newLatLng);

                path.push(newLatLng);
                pathLine.setLatLngs(path);
                savePathToLocalStorage();
            }
        };

        map.invalidateSize();

        return () => {
            if (socket) {
                socket.close();
            }
        };
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
        const response = await fetch(`http://${host}:8000/config`);
        const data = await response.json();
        topics = data.topics;

        const allowedTopics = [
            "mavlink/simba_max_altitude",
            "mavlink/simba_altitude_orientation",
            "mavlink/simba_gps"
        ];

        topics = topics.filter(topic => allowedTopics.includes(topic.name));

        console.log(topics);
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
                <TelemetryField
                    {topic}
                    {host}
                />
            {/each}
        </div>
        <!-- <div class="field">
            <div
                class="status-indicator {data !== 'None' &&
                data !== null &&
                data !== undefined
                    ? 'green-status'
                    : 'red-status'}"
            ></div>
            <div class="field-content">
                <span class="field-text">LOCATION</span>
                {#if data != undefined && data !== "None" && data !== null}
                    <span class="timestamp"
                        >{rosTimeToFormattedTime(
                            data.header.stamp.sec,
                            data.header.stamp.nanosec,
                        )}</span
                    >
                    <div class="field-value">
                        <span>Altitude:</span>
                        <span>{data.altitude}</span>
                    </div>
                    <div class="field-value">
                        <span>Longitude:</span>
                        <span>{data.longitude}</span>
                    </div>
                    <div class="field-value">
                        <span>Latitude:</span>
                        <span>{data.latitude}</span>
                    </div>
                {/if}
            </div>
        </div>
        <div class="field">
            <div
                class="status-indicator {data !== 'None' &&
                data !== null &&
                data !== undefined
                    ? 'green-status'
                    : 'red-status'}"
            ></div>
            <div class="field-content">
                <span class="field-text">MAX ALTITUDE</span>
                {#if data != undefined && data !== "None" && data !== null}
                    <span class="timestamp"
                        >{rosTimeToFormattedTime(
                            data.header.stamp.sec,
                            data.header.stamp.nanosec,
                        )}</span
                    >
                    <div class="field-value">
                        <span>Max altitude:</span>
                        <span>{data.altitude}</span>
                    </div>
                {/if}
            </div>
        </div> -->
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
        gap: 10px;
        /* margin-bottom: 16px; */
        justify-content: space-between;
    }

    .button {
        padding: 10px 16px;
        border: none;
        border-radius: 0.2rem;
        cursor: pointer;
        display: inline-flex;
        align-items: center;
        font-weight: 500;
        cursor: pointer;
        background: rgb(61, 113, 217);
        width: 110px;
        justify-content: center;
        text-align: center;
    }

    .button:hover {
        background-color: rgb(90, 134, 222);
        transition:
            background-color 0.3s,
            color 0.3s;
    }

    .info-widget {
        position: absolute;
        top: calc(var(--navbar-height) + 20px);
        left: 75%;
        width: 20%;
        height: 30%;
        padding: 16px;
        border: 1px solid rgba(204, 204, 220, 0.5);
        border-radius: 0.75rem;
        background-color: #111217;
        color: inherit;
        z-index: 1000;
        display: flex;
        flex-direction: column;
        justify-content: flex-start;
        overflow-y: auto;
        line-height: 1.5rem;
        font-size: 0.9rem;
        box-shadow: rgba(0, 0, 0, 0.24) 0px 3px 8px;
    }

    .field {
        display: flex;
        align-items: flex-start;
        border-bottom: 1px solid rgba(204, 204, 220, 0.5);
        text-align: left;
    }

    .field-content {
        flex: 1;
    }

    .field:last-child {
        border-bottom: none;
    }

    .field:first-child {
        border-top: 1px solid rgba(204, 204, 220, 0.5);
    }

    .field-text {
        color: #ccccdc;
    }

    .field-value span:first-child {
        font-weight: bold;
        margin-right: 4px;
    }

    .status-indicator {
        width: 16px;
        height: 16px;
        border-radius: 50%;
        margin-right: 8px;
    }

    .timestamp {
        margin-left: 8px;
        color: rgba(204, 204, 220, 0.65);
        font-size: 0.8em;
    }

    .green-status {
        background: linear-gradient(90deg, #7fff7f, #5eff5e, #3dff3d, #1aff1a);
        background-size: 100% 100%;
        animation: gradientAnimation 3s ease infinite;
    }

    .orange-status {
        background: linear-gradient(45deg, orange, yellow);
        background-size: 100% 100%;
        animation: orangeGradientAnimation 3s ease infinite;
    }

    .red-status {
        background: linear-gradient(45deg, red, pink);
        background-size: 100% 100%;
        animation: redGradientAnimation 3s ease infinite;
    }
</style>
