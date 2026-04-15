<script>
    import { onMount, onDestroy } from "svelte";
    import TelemetryField from "../components/fields/TelemetryField.svelte";
    // @ts-ignore
    import L from "leaflet";
    import { queryFieldData } from "../services/api.js";

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
        Tricity: {
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
            center: [35.27997, -117.813],
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
        map.setView(config.center, config.maxZoom - 2);

        selectedMap = mapName;
        localStorage.setItem("selectedMap", mapName);
    };

    const customIcon = L.icon({
        iconUrl: "icons/simba_logo.png",
        iconSize: [38, 38],
        iconAnchor: [19, 38],
        popupAnchor: [0, -38],
    });

    let timeRange = localStorage.getItem("map_time_range") || "1"; // 1 minute default

    const setTimeRange = (range) => {
        timeRange = range;
        localStorage.setItem("map_time_range", range);
        clearPath();
        loadHistoricalPath();
    };

    const loadHistoricalPath = async () => {
        try {
            const rawLatData = await queryFieldData(
                "mavlink/simba_gps",
                "lat",
                timeRange,
            );
            const rawLonData = await queryFieldData(
                "mavlink/simba_gps",
                "lon",
                timeRange,
            );

            const latArray = rawLatData.records || [];
            const lonArray = rawLonData.records || [];

            if (
                latArray.length > 0 &&
                lonArray.length > 0 &&
                latArray.length === lonArray.length
            ) {
                path = latArray.map((latPoint, i) => {
                    const lat = latPoint._value;
                    const lon = lonArray[i]._value;
                    return [lat, lon];
                });
            } else {
                console.warn(
                    "Could not parse GPS data. Arrays were empty or lengths mismatched.",
                );
            }

            if (path.length > 0) {
                pathLine.setLatLngs(path);
                const latestPoint = path[path.length - 1];
                marker.setLatLng(latestPoint);
            }
        } catch (error) {
            console.error("Error loading historical GPS path from DB:", error);
        }
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

        pathLine = L.polyline(path, { color: "orange" }).addTo(map);
        loadHistoricalPath();

        gpsSocket = new WebSocket(
            `ws://${window.location.host}/mavlink/simba_gps`,
        );
        gpsSocket.onmessage = (event) => {
            try {
                gpsData = JSON.parse(event.data);
                if (
                    gpsData !== null &&
                    gpsData !== undefined &&
                    gpsData.lat &&
                    gpsData.lon
                ) {
                    const newLatLng = [gpsData.lat, gpsData.lon];
                    marker.setLatLng(newLatLng);

                    path.push(newLatLng);
                    pathLine.setLatLngs(path);
                    // LocalStorage save removed here
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
        if (gpsSocket) gpsSocket.close();
        if (maxAltitudeSocket) maxAltitudeSocket.close();
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
    };

    async function fetchConfig() {
        try {
            const response = await fetch(
                `http://${window.location.host}/config`,
            );
            const data = await response.json();

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

        <div class="time-menu">
            <label>Path History:</label>
            <div class="time-controls">
                <button
                    class:selected={timeRange === "1"}
                    on:click={() => setTimeRange("1")}>1m</button
                >
                <button
                    class:selected={timeRange === "10"}
                    on:click={() => setTimeRange("10")}>10m</button
                >
                <button
                    class:selected={timeRange === "30"}
                    on:click={() => setTimeRange("30")}>30m</button
                >
                <button
                    class:selected={timeRange === "60"}
                    on:click={() => setTimeRange("60")}>1h</button
                >
                <button
                    class:selected={timeRange === "120"}
                    on:click={() => setTimeRange("120")}>2h</button
                >
            </div>
        </div>

        <hr class="divider" />

        <div class="buttons">
            <button class="button" on:click={togglePath} >
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
        margin: 20px 0 20px 0;
    }

    .buttons {
        display: flex;
        align-items: center;
        gap: 25px;
        justify-content: center;
    }

    .button {
        padding: 10px 16px;
        border-radius: 0.2rem;
        cursor: pointer;
        display: inline-flex;
        align-items: center;
        font-weight: 500;
        background-color: var(--bg-color);
        color: var(--text-color);
        border: 1px solid var(--border-color);
        width: 160px;
        justify-content: center;
        text-align: center;
        font-size: 1rem;
        transition:
            background-color 0.2s,
            color 0.2s,
            border-color 0.2s;
    }

    .buttons button:hover:not(.selected) {
        background-color: var(--snd-bg-color);
        border: 1px solid #ff965f;
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
        right: 20px;
        
        width: 90%;
        min-width: 280px;
        max-width: 300px;

        height: auto;
        min-height: 350px;
        max-height: calc(100vh - var(--navbar-height) - 40px);

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

    .time-menu {
        display: flex;
        flex-direction: column;
        gap: 5px;
    }

    .time-controls {
        display: flex;
        gap: 4px;
        flex-wrap: wrap;
    }

    .time-controls button {
        font-size: 0.8rem;
        background-color: var(--bg-color);
        color: var(--text-color);
        border: 1px solid var(--border-color);
        border-radius: 3px;
        cursor: pointer;
        padding: 4px 6px;
        flex: 1;
        text-align: center;
        transition:
            background-color 0.2s,
            color 0.2s;
    }

    .time-controls button.selected {
        background-color: #ff965f;
        border-color: #ff965f;
        color: #181b1f;
    }

    .time-controls button:hover:not(.selected) {
        background-color: var(--snd-bg-color);
    }
</style>
