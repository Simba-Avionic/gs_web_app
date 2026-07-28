<script>
    import { onMount, onDestroy } from "svelte";

    // @ts-ignore
    import L from "leaflet";
    import { queryFieldData } from "../services/api.js";

    let topics = [];

    let map;
    let currentLayer;
    let selectedMap = localStorage.getItem("selectedMap") || "Mojave Desert";

    // Load saved source names or fall back to defaults
    const savedNames = JSON.parse(
        localStorage.getItem("gps_source_names") || "{}",
    );

    let gpsSources = [
        {
            id: "simba_main",
            name: savedNames["simba_main"] || "Main GPS",
            topic: "mavlink/simba_gps",
            latField: "lat",
            lonField: "lon",
            color: "#ff965f",
            opacity: 1.0,
            zIndex: 1000,
            visible: true,
            path: [],
            marker: null,
            polyline: null,
            socket: null,
            currentCoords: { lat: "-", lon: "-", alt: "-", time: "-" },
        },
        {
            id: "backup_gps",
            name: savedNames["backup_gps"] || "LoRa GPS",
            topic: "rocket/lora/gps",
            latField: "latitude_deg",
            lonField: "longitude_deg",
            color: "#3388ff",
            opacity: 0.8,
            zIndex: 900,
            visible: true,
            path: [],
            marker: null,
            polyline: null,
            socket: null,
            currentCoords: { lat: "-", lon: "-", alt: "-", time: "-" },
        },
    ];

    let activeSourceId = gpsSources[0].id;
    $: activeSource = gpsSources.find((s) => s.id === activeSourceId);

    // --- Custom Markers State Engine ---
    let customMarkers = JSON.parse(
        localStorage.getItem("custom_map_markers") || "[]",
    );
    let isPlacingMarkerMode = false;
    let manualLat = "";
    let manualLon = "";
    let manualLabel = "";

    let maxAltitudeSocket;
    let maxAltitudeData;

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
            center: [35.3337997, -117.813],
            minZoom: 13,
            maxZoom: 17,
        },
        Lancaster: {
            url: `http://${window.location.host}/tiles/lancaster/{z}/{x}/{y}.png`,
            center: [34.70485, -118.079],
            minZoom: 11,
            maxZoom: 16,
            bounds: [
                [34.6, -118.15],
                [34.78, -117.9],
            ],
        },
    };

    const switchMap = (mapName) => {
        const config = mapConfigs[mapName];
        if (!config || !map) return;

        if (currentLayer) {
            map.removeLayer(currentLayer);
        }

        map.setView(config.center, config.maxZoom - 2);

        const layerOptions = {
            minZoom: config.minZoom,
            maxZoom: config.maxZoom,
            tileSize: 256,
            tms: false,
        };

        if (config.bounds) {
            layerOptions.bounds = config.bounds;
        }

        currentLayer = L.tileLayer(config.url, layerOptions);
        currentLayer.addTo(map);

        selectedMap = mapName;
        localStorage.setItem("selectedMap", mapName);
    };

    const createCustomIcon = (color) => {
        return L.divIcon({
            className: "custom-gps-pin",
            html: `<div style="background-color: ${color}; width: 14px; height: 14px; border-radius: 50%; border: 2px solid white; box-shadow: 0 0 4px rgba(0,0,0,0.5);"></div>`,
            iconSize: [14, 14],
            iconAnchor: [7, 7],
        });
    };

    const createPinnedIcon = () => {
        return L.divIcon({
            className: "pinned-custom-marker",
            html: `<div style="background-color: #e74c3c; width: 12px; height: 12px; border-radius: 50%; border: 2px solid #fff; box-shadow: 0 0 6px rgba(0,0,0,0.6); position: relative;">
                     <div style="position: absolute; top: -18px; left: 50%; transform: translateX(-50%); background: #2c3e50; color: #fff; font-size: 10px; padding: 2px 4px; border-radius: 3px; white-space: nowrap;">Pin</div>
                   </div>`,
            iconSize: [12, 12],
            iconAnchor: [6, 6],
        });
    };

    let timeRange = localStorage.getItem("map_time_range") || "1";

    const setTimeRange = (range) => {
        timeRange = range;
        localStorage.setItem("map_time_range", range);
        gpsSources.forEach((source) => clearSourcePath(source));
        loadAllHistoricalPaths();
    };

    const loadAllHistoricalPaths = async () => {
        for (let source of gpsSources) {
            try {
                const rawLatData = await queryFieldData(
                    source.topic,
                    source.latField,
                    timeRange,
                );
                const rawLonData = await queryFieldData(
                    source.topic,
                    source.lonField,
                    timeRange,
                );

                const latArray = rawLatData.records || [];
                const lonArray = rawLonData.records || [];

                if (
                    latArray.length > 0 &&
                    lonArray.length > 0 &&
                    latArray.length === lonArray.length
                ) {
                    source.path = latArray.map((latPoint, i) => {
                        return [latPoint._value, lonArray[i]._value];
                    });

                    if (source.polyline && source.visible) {
                        source.polyline.setLatLngs(source.path);
                    }
                    if (source.marker && source.path.length > 0) {
                        source.marker.setLatLng(
                            source.path[source.path.length - 1],
                        );
                    }

                    const lastIdx = latArray.length - 1;
                    source.currentCoords.lat =
                        latArray[lastIdx]._value.toFixed(5);
                    source.currentCoords.lon =
                        lonArray[lastIdx]._value.toFixed(5);

                    if (latArray[lastIdx]._time) {
                        const dateObj = new Date(latArray[lastIdx]._time);
                        source.currentCoords.time = dateObj
                            .toTimeString()
                            .split(" ")[0];
                    }
                }
            } catch (error) {
                console.error(
                    `Error loading history for ${source.name}:`,
                    error,
                );
            }
        }
        gpsSources = gpsSources;
    };

    const updateStyle = (source) => {
        if (source.polyline) {
            source.polyline.setStyle({
                color: source.color,
                opacity: source.opacity,
            });
        }
        if (source.marker) {
            source.marker.setIcon(createCustomIcon(source.color));
            if (source.marker.getElement()) {
                source.marker.setZIndexOffset(source.zIndex);
            }
        }
    };

    const handleNameChange = (source) => {
        const namesMap = {};
        gpsSources.forEach((s) => {
            namesMap[s.id] = s.name;
        });
        localStorage.setItem("gps_source_names", JSON.stringify(namesMap));
        gpsSources = gpsSources; // Trigger reactivity
    };

    const toggleSourceVisibility = (source) => {
        source.visible = !source.visible;
        if (source.visible) {
            if (source.polyline) source.polyline.addTo(map);
            if (source.marker) source.marker.addTo(map);
        } else {
            if (source.polyline) source.polyline.removeFrom(map);
            if (source.marker) source.marker.removeFrom(map);
        }
        gpsSources = gpsSources;
    };

    const clearSourcePath = (source) => {
        source.path = [];
        if (source.polyline) source.polyline.setLatLngs([]);
        gpsSources = gpsSources;
    };

    // --- Custom Markers Leaflet Engine ---
    const addCustomMarkerToMap = (markerData) => {
        const m = L.marker([markerData.lat, markerData.lon], {
            icon: createPinnedIcon(),
        }).addTo(map);

        m.bindPopup(
            `<b>${markerData.label || "Custom Location"}</b><br>Lat: ${markerData.lat}<br>Lon: ${markerData.lon}`,
        );
        markerData._leafletMarker = m;
    };

    const handleMapClick = (e) => {
        if (!isPlacingMarkerMode) return;

        const { lat, lng } = e.latlng;
        const label = prompt(
            "Enter a label for this custom point:",
            `Marker ${customMarkers.length + 1}`,
        );
        if (label === null) return; // Cancelled

        const newMarker = {
            id: Math.random().toString(36).substr(2, 9),
            lat: parseFloat(lat.toFixed(6)),
            lon: parseFloat(lng.toFixed(6)),
            label: label || `Marker ${customMarkers.length + 1}`,
        };

        customMarkers = [...customMarkers, newMarker];
        localStorage.setItem(
            "custom_map_markers",
            JSON.stringify(
                customMarkers.map(({ _leafletMarker, ...rest }) => rest),
            ),
        );
        addCustomMarkerToMap(newMarker);
        isPlacingMarkerMode = false;
    };

    const addManualMarker = () => {
        const latVal = parseFloat(manualLat);
        const lonVal = parseFloat(manualLon);
        if (isNaN(latVal) || isNaN(lonVal)) {
            alert("Please input valid numeric coordinates.");
            return;
        }

        const newMarker = {
            id: Math.random().toString(36).substr(2, 9),
            lat: latVal,
            lon: lonVal,
            label:
                manualLabel.trim() ||
                `Manual Marker ${customMarkers.length + 1}`,
        };

        customMarkers = [...customMarkers, newMarker];
        localStorage.setItem(
            "custom_map_markers",
            JSON.stringify(
                customMarkers.map(({ _leafletMarker, ...rest }) => rest),
            ),
        );
        addCustomMarkerToMap(newMarker);

        // Clear manual inputs
        manualLat = "";
        manualLon = "";
        manualLabel = "";
    };

    const deleteCustomMarker = (id) => {
        const index = customMarkers.findIndex((m) => m.id === id);
        if (index !== -1) {
            if (customMarkers[index]._leafletMarker) {
                map.removeLayer(customMarkers[index]._leafletMarker);
            }
            customMarkers.splice(index, 1);
            customMarkers = [...customMarkers];
            localStorage.setItem(
                "custom_map_markers",
                JSON.stringify(
                    customMarkers.map(({ _leafletMarker, ...rest }) => rest),
                ),
            );
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

        gpsSources.forEach((source) => {
            source.polyline = L.polyline([], {
                color: source.color,
                opacity: source.opacity,
                weight: 4,
            }).addTo(map);

            source.marker = L.marker(initialCoords, {
                icon: createCustomIcon(source.color),
            }).addTo(map);
        });

        loadAllHistoricalPaths();

        // Render Persistent Custom Markers on Mount
        customMarkers.forEach((markerData) => {
            addCustomMarkerToMap(markerData);
        });

        map.on("click", handleMapClick);

        gpsSources.forEach((source, index) => {
            source.socket = new WebSocket(
                `ws://${import.meta.env.VITE_BACKEND_URL}/${source.topic}`,
            );
            source.socket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (
                        data &&
                        data[source.latField] &&
                        data[source.lonField]
                    ) {
                        const lat = data[source.latField];
                        const lon = data[source.lonField];
                        const alt =
                            data.alt || data.altitude || data.altitude_m || "-";

                        const newLatLng = [lat, lon];
                        const now = new Date();
                        const timeString = now.toTimeString().split(" ")[0];

                        source.currentCoords = {
                            lat: lat.toFixed(5),
                            lon: lon.toFixed(5),
                            alt: typeof alt === "number" ? alt.toFixed(1) : alt,
                            time: timeString,
                        };

                        source.path.push(newLatLng);

                        if (source.visible) {
                            source.polyline.setLatLngs(source.path);
                            source.marker.setLatLng(newLatLng);
                        }
                        gpsSources[index] = source;
                    }
                } catch (e) {
                    console.error(
                        `Error processing WS stream for ${source.name}:`,
                        e,
                    );
                }
            };
        });

        map.invalidateSize();
    });

    onDestroy(() => {
        gpsSources.forEach((source) => {
            if (source.socket) source.socket.close();
        });
        if (map) {
            map.off("click", handleMapClick);
        }
    });

    async function fetchConfig() {
        try {
            const response = await fetch(
                `http://${import.meta.env.VITE_BACKEND_URL}/config`,
            );
            const data = await response.json();
            const allowedTopics = [...gpsSources.map((s) => s.topic)];

            topics = data.topics.filter((topic) =>
                allowedTopics.includes(topic.topic_name || topic.name),
            );
            topics = topics.map((topic) => ({
                id: topic.id || Math.random().toString(),
                topic_name: topic.topic_name || topic.name,
                msg_fields: topic.msg_fields || [],
            }));
        } catch (error) {
            console.error("Error fetching config:", error);
        }
    }
</script>

<div id="map">
    <div class="info-widget">
        <div class="top-settings-grid">
            <div class="map-menu">
                <label for="map-select">Active Map Base Layer:</label>
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

            <div class="time-menu">
                <label for="time-span-ui">Path History:</label>
                <div id="time-span-ui" class="time-controls">
                    {#each ["1", "10", "30", "60", "120"] as range}
                        <button
                            class:selected={timeRange === range}
                            on:click={() => setTimeRange(range)}
                        >
                            {range >= 60 ? `${range / 60}h` : `${range}m`}
                        </button>
                    {/each}
                </div>
            </div>
        </div>

        <hr class="divider" />

        <div class="sources-dashboard">
            <label>GPS Statuses:</label>
            {#each gpsSources as source}
                <div class="source-item-row" class:muted={!source.visible}>
                    <div class="source-header">
                        <span
                            class="color-indicator"
                            style="background-color: {source.color}"
                        ></span>
                        <input
                            type="text"
                            class="source-name-input"
                            bind:value={source.name}
                            on:change={() => handleNameChange(source)}
                        />
                        <span class="source-timestamp"
                            >[{source.currentCoords.time}]</span
                        >
                        <button
                            class="visibility-btn"
                            on:click={() => toggleSourceVisibility(source)}
                        >
                            {source.visible ? "Hide" : "Show"}
                        </button>
                    </div>
                    {#if source.visible}
                        <div class="source-coords-grid">
                            <span>Lat: {source.currentCoords.lat}°</span>
                            <span>Lon: {source.currentCoords.lon}°</span>
                            <span>Alt: {source.currentCoords.alt}m</span>
                        </div>
                    {/if}
                </div>
            {/each}
        </div>

        <hr class="divider" />

        <div class="custom-markers-section">
            <label>Custom Map Markers:</label>

            <div class="manual-marker-form">
                <input
                    type="text"
                    placeholder="Label/Name"
                    bind:value={manualLabel}
                />
                <div class="coords-row">
                    <input
                        type="number"
                        step="0.000001"
                        placeholder="Latitude"
                        bind:value={manualLat}
                    />
                    <input
                        type="number"
                        step="0.000001"
                        placeholder="Longitude"
                        bind:value={manualLon}
                    />
                </div>
                <button class="add-manual-btn" on:click={addManualMarker}
                    >Drop Pin</button
                >
            </div>

            {#if customMarkers.length > 0}
                <div class="markers-list">
                    {#each customMarkers as marker}
                        <div class="marker-list-item">
                            <span class="marker-item-info">
                                <strong>{marker.label}</strong> <br />
                                <small>({marker.lat}, {marker.lon})</small>
                            </span>
                            <button
                                class="delete-marker-btn"
                                on:click={() => deleteCustomMarker(marker.id)}
                                >✕</button
                            >
                        </div>
                    {/each}
                </div>
            {/if}
        </div>

        <hr class="divider" />

        {#if activeSource}
            <details class="style-accordion">
                <summary class="style-summary">Line Styles & Overrides</summary>
                <div class="style-panel">
                    <div class="style-selection-row">
                        <select
                            id="source-style-select"
                            bind:value={activeSourceId}
                        >
                            {#each gpsSources as src}
                                <option value={src.id}>{src.name}</option>
                            {/each}
                        </select>
                        <button
                            class="clear-btn"
                            on:click={() =>
                                gpsSources.forEach((s) => clearSourcePath(s))}
                        >
                            Clear Paths
                        </button>
                    </div>

                    <div class="horizontal-controls">
                        <div class="control-item shrink-fix">
                            <span class="control-label">Line Color</span>
                            <input
                                type="color"
                                bind:value={activeSource.color}
                                on:input={() => updateStyle(activeSource)}
                            />
                        </div>

                        <div class="control-item">
                            <span class="control-label"
                                >Opacity ({activeSource.opacity})</span
                            >
                            <input
                                type="range"
                                min="0.1"
                                max="1.0"
                                step="0.1"
                                bind:value={activeSource.opacity}
                                on:input={() => updateStyle(activeSource)}
                            />
                        </div>

                        <div class="control-item">
                            <span class="control-label"
                                >Z-Index ({activeSource.zIndex})</span
                            >
                            <input
                                type="range"
                                min="100"
                                max="2000"
                                step="50"
                                bind:value={activeSource.zIndex}
                                on:input={() => updateStyle(activeSource)}
                            />
                        </div>
                    </div>
                </div>
            </details>
        {/if}
    </div>
</div>

<style>
    #map {
        height: 100vh;
        width: 100%;
        position: relative;
    }

    .top-settings-grid {
        display: grid;
        grid-template-columns: 140px 1fr;
        gap: 8px;
        align-items: flex-start;
    }

    .map-menu,
    .style-panel,
    .sources-dashboard,
    .time-menu,
    .custom-markers-section {
        display: flex;
        flex-direction: column;
        gap: 5px;
    }

    .map-menu select,
    .style-panel select {
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
        margin: 15px 0;
    }

    .source-item-row {
        background: var(--snd-bg-color);
        border: 1px solid var(--border-color);
        border-radius: 6px;
        padding: 8px;
        margin-bottom: 6px;
        transition: opacity 0.2s;
    }
    .source-item-row.muted {
        opacity: 0.5;
    }
    .source-header {
        display: flex;
        align-items: center;
        gap: 8px;
        font-weight: 600;
    }
    .source-name-input {
        flex: 1;
        background: transparent;
        border: 1px dashed transparent;
        color: var(--text-color);
        font-weight: 600;
        font-size: 0.9rem;
        padding: 2px 4px;
        border-radius: 3px;
    }
    .source-name-input:focus,
    .source-name-input:hover {
        background: var(--bg-color);
        border-color: var(--border-color);
        outline: none;
    }
    .color-indicator {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        display: inline-block;
        flex-shrink: 0;
    }
    .source-timestamp {
        font-size: 0.75rem;
        font-weight: 400;
        opacity: 0.6;
        white-space: nowrap;
    }
    .visibility-btn,
    .clear-btn,
    .map-pick-btn,
    .add-manual-btn {
        background: var(--snd-bg-color);
        border: 1px solid var(--border-color);
        color: var(--text-color);
        padding: 4px 8px;
        border-radius: 4px;
        cursor: pointer;
        font-size: 0.75rem;
    }
    .clear-btn {
        padding: 8px 12px;
    }
    .clear-btn:hover {
        border-color: #ff965f;
    }
    .source-coords-grid {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        font-size: 0.75rem;
        margin-top: 5px;
        color: var(--text-color);
        opacity: 0.9;
    }

    /* Manual Marker Form CSS Styles */
    .manual-marker-form {
        display: flex;
        flex-direction: column;
        gap: 4px;
        margin-top: 4px;
        background: var(--snd-bg-color);
        padding: 6px;
        border-radius: 4px;
        border: 1px solid var(--border-color);
    }
    .manual-marker-form input {
        background: var(--bg-color);
        color: var(--text-color);
        border: 1px solid var(--border-color);
        padding: 4px;
        border-radius: 3px;
        font-size: 0.75rem;
    }
    .manual-marker-form .coords-row {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 4px;
    }
    .add-manual-btn {
        background: var(--bg-color);
    }

    /* Custom Marker Items List styling */
    .markers-list {
        max-height: 110px;
        overflow-y: auto;
        margin-top: 6px;
        border: 1px solid var(--border-color);
        border-radius: 4px;
        background: var(--snd-bg-color);
    }
    .marker-list-item {
        display: flex;
        justify-content: space-between;
        align-items: center;
        padding: 4px 8px;
        font-size: 0.75rem;
        border-bottom: 1px solid var(--border-color);
    }
    .marker-list-item:last-child {
        border-bottom: none;
    }
    .marker-item-info small {
        opacity: 0.6;
    }
    .delete-marker-btn {
        background: transparent;
        border: none;
        color: #e74c3c;
        cursor: pointer;
        font-weight: bold;
        font-size: 0.8rem;
    }

    .style-accordion {
        background: var(--snd-bg-color);
        border: 1px solid var(--border-color);
        border-radius: 6px;
        padding: 6px 10px;
    }
    .style-summary {
        font-size: 0.8rem;
        font-weight: 600;
        cursor: pointer;
        user-select: none;
        color: var(--text-color);
    }
    .style-panel {
        margin-top: 8px;
        padding-top: 8px;
        border-top: 1px dashed var(--border-color);
    }
    .style-selection-row {
        display: grid;
        grid-template-columns: 1fr auto;
        gap: 8px;
        align-items: center;
        margin-bottom: 8px;
    }

    .horizontal-controls {
        display: flex;
        gap: 10px;
        align-items: center;
        justify-content: space-between;
        background: var(--bg-color);
        padding: 8px;
        border-radius: 4px;
        border: 1px solid var(--border-color);
    }
    .control-item {
        display: flex;
        flex-direction: column;
        align-items: center;
        gap: 4px;
        flex: 1;
    }
    .control-item.shrink-fix {
        flex: 0 0 auto;
    }
    .control-label {
        font-size: 0.75rem;
        opacity: 0.8;
        white-space: nowrap;
    }
    .control-item input[type="color"] {
        border: none;
        width: 30px;
        height: 24px;
        cursor: pointer;
        background: transparent;
        padding: 0;
    }
    .control-item input[type="range"] {
        width: 100%;
    }

    .info-widget {
        position: absolute;
        top: calc(var(--navbar-height) + 20px);
        right: 20px;
        width: 350px;
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
        box-shadow: rgba(0, 0, 0, 0.24) 0px 3px 8px;
    }

    .time-controls {
        display: flex;
        gap: 4px;
        width: 100%;
    }

    .time-controls button {
        font-size: 0.8rem;
        background-color: var(--bg-color);
        color: var(--text-color);
        border: 1px solid var(--border-color);
        border-radius: 3px;
        cursor: pointer;
        padding: 6px;
        flex: 1;
        text-align: center;
    }

    .time-controls button.selected {
        background-color: #ff965f;
        border-color: #ff965f;
        color: #181b1f;
    }
</style>
