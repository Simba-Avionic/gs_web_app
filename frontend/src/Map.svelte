<!-- MapWidget.svelte -->
<script>
    import { onMount, onDestroy } from "svelte";
    import TelemetryField from "./lib/TelemetryField.svelte";
    // @ts-ignore
    import L from "leaflet";

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

    let currentLayer;
    let selectedMap = localStorage.getItem("selectedMap") || "Tricity";

    const mapConfigs = {
        "Tricity": {
            url: `http://${window.location.host}/tiles/tricity/{z}/{x}/{y}.png`,
            center: [54.4034, 18.5166],
            minZoom: 10,
            maxZoom: 15,
        },
        "Drawsko Pomorskie": {
            url: `http://${window.location.host}/tiles/drawsko/{z}/{x}/{y}.png`,
            center: [53.4781, 15.727],
            minZoom: 10,
            maxZoom: 15,
        },
        "Mojave Desert": {
            url: `http://${window.location.host}/tiles/mojave/{z}/{x}/{y}.png`,
            center: [35.0846, -115.5242],
            minZoom: 10,
            maxZoom: 15,
        },
    };

    const switchMap = (mapName) => {
        const config = mapConfigs[mapName];
        if (!config || !map) return;

        if (currentLayer) {
            map.removeLayer(currentLayer);
        }

        currentLayer = L.tileLayer(config.url, {
            minZoom: config.minZoom,
            maxZoom: config.maxZoom,
            tileSize: 256,
            tms: false,
        });

        currentLayer.addTo(map);

        map.setView(config.center, config.minZoom);

        selectedMap = mapName;
        localStorage.setItem("selectedMap", mapName);
    };

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
        });

        switchMap(selectedMap);

        L.control.zoom({ position: "bottomright" }).addTo(map);

        const initialCoords = mapConfigs[selectedMap].center;
        marker = L.marker(initialCoords, { icon: customIcon }).addTo(map);

        path = loadPathFromLocalStorage();
        pathLine = L.polyline(path, { color: "orange" }).addTo(map);

        gpsSocket = new WebSocket(
            `ws://${window.location.host}/mavlink/simba_gps`,
        );
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

        maxAltitudeSocket = new WebSocket(
            `ws://${window.location.host}/mavlink/simba_max_altitude`,
        );
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
            const response = await fetch(
                `http://${window.location.host}/config`,
            );
            const data = await response.json();

            // Filter topics to only include GPS and max altitude topics
            // TODO: Do it in config.json
            const allowedTopics = [
                "mavlink/simba_max_altitude",
                "mavlink/simba_gps",
            ];

            topics = data.topics.filter((topic) =>
                allowedTopics.includes(topic.topic_name || topic.name),
            );

            topics = topics.map((topic) => {
                return {
                    id: topic.id || Math.random().toString(),
                    topic_name: topic.topic_name || topic.name,
                    msg_fields: topic.msg_fields || [],
                };
            });
        } catch (error) {
            console.error("Error fetching config:", error);
        }
    }
</script>

<div id="map">
    <div class="info-widget">
        <div class="map-menu">
            <label for="map-select">Active Map:</label>
            <select
                id="map-select"
                bind:value={selectedMap}
                on:change={() => switchMap(selectedMap)}
            >
                {#each Object.keys(mapConfigs) as name}
                    <option value={name}>{name}</option>
                {/each}
            </select>
        </div>

        <hr class="divider" />

        <div class="buttons">
            <button class="button" on:click={togglePath}>
                {showPath ? "Hide Path" : "Show Path"}
            </button>
            <button class="button" on:click={clearPath}>Clear Path</button>
        </div>

        <div class="fields-container">
            {#each topics as topic (topic.id)}
                <TelemetryField {topic} />
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
    .map-menu {
        display: flex;
        flex-direction: column;
        gap: 5px;
        margin-bottom: 15px;
    }

    .map-menu select {
        padding: 8px;
        background: var(--snd-bg-color);
        color: var(--text-color);
        border: 1px solid var(--border-color);
        border-radius: 4px;
        cursor: pointer;
    }

    .divider {
        border: 0;
        border-top: 1px solid var(--border-color);
        margin: 10px 0 20px 0;
        opacity: 0.55;
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
