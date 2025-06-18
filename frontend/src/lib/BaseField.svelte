<!-- BaseField.svelte -->
<script>
    import { onMount, onDestroy, createEventDispatcher } from "svelte";
    import { slide } from "svelte/transition";
    import { rosTimeToFormattedTime } from "./Utils.svelte";

    // Common props
    export let dataWarning = false;
    export let className; // Accept custom classes
    export let title;
    export let host;
    export let topicNames = []; // List of topics to subscribe to
    export let isExpanded = false;
    export let processData = (topicName, data) => {}; // Override this in child components

    // Common state
    const dispatch = createEventDispatcher();
    let topicStatusMap = {};
    let lastUpdated = null;
    let hasError = false;
    let hasWarning = false;
    let sockets = {};
    let telemetryData = {};

    function toggleExpand() {
        isExpanded = !isExpanded;
    }

    //   let initStartTime;

    function setupWebSockets() {
        // initStartTime = performance.now();

        // Create all WebSocket instances immediately instead of sequentially
        const connectionPromises = topicNames.map((topicName) => {
            return new Promise((resolve) => {
                topicStatusMap[topicName] = false;

                const socket = new WebSocket(`ws://${host}:8000/${topicName}`);

                socket.onopen = () => {
                    console.log(`Connected to ${topicName}`);
                    topicStatusMap[topicName] = false;
                    resolve();
                };

                socket.onmessage = (event) => {
                    try {
                        const data = JSON.parse(event.data);
                        if (
                            data !== "None" &&
                            data !== null &&
                            data !== undefined
                        ) {
                            // Store raw data
                            telemetryData[topicName] = data;

                            // Process data (implemented by child component)
                            processData(topicName, data);

                            // Update timestamp
                            try {
                                lastUpdated = {
                                    sec: data.header.stamp.sec,
                                    nanosec: data.header.stamp.nanosec,
                                };
                            } catch (e) {
                                // Silent catch - not all messages have headers
                            }

                            // Only update connection status if it was previously false
                            if (!topicStatusMap[topicName]) {
                                topicStatusMap[topicName] = true;
                                updateStatus();
                            }

                            dispatch("dataReceived", { topicName, data });
                            dispatch("telemetryChange", { data });
                        } else {
                            if (topicStatusMap[topicName]) {
                                topicStatusMap[topicName] = false;
                                updateStatus();
                            }
                        }
                    } catch (e) {
                        console.error(
                            `Error processing data from ${topicName}:`,
                            e,
                        );
                    }
                };

                socket.onclose = () => {
                    console.log(`Disconnected from ${topicName}`);
                    if (topicStatusMap[topicName]) {
                        topicStatusMap[topicName] = false;
                        updateStatus();
                    }
                    resolve();
                };

                sockets[topicName] = socket;
            });
        });

        // Update status once all connections are attempted
        Promise.all(connectionPromises).then(() => {
            updateStatus();
            //   const elapsed = performance.now() - initStartTime;
            //   console.log(`All WebSocket connections initialized in ${elapsed.toFixed(2)}ms`);
        });
    }

    let statusUpdateTimeout;
    function updateStatus() {
        clearTimeout(statusUpdateTimeout);
        statusUpdateTimeout = setTimeout(() => {
            // Update status indicators
            const allOffline =
                topicNames.length > 0 &&
                topicNames.every((topic) => !topicStatusMap[topic]);

            const someOffline =
                topicNames.some((topic) => !topicStatusMap[topic]) &&
                !allOffline;

            hasError = allOffline;
            hasWarning = someOffline || dataWarning;

            dispatch("statusChange", { hasError, hasWarning });
        }, 5); // Small timeout to batch multiple rapid status changes
    }

    $: if (dataWarning !== undefined) {
        updateStatus();
    }

    onMount(() => {
        setupWebSockets();
    });

    onDestroy(() => {
        clearTimeout(statusUpdateTimeout);
        Object.values(sockets).forEach((socket) => {
            if (socket && socket.readyState < 2) {
                socket.close();
            }
        });
    });
</script>

<div class="field {className}">
    <div class="field-top-row">
        <!-- Status indicator based on error/warning state -->
        <div
            class="status-indicator {hasError
                ? 'red-status'
                : hasWarning
                  ? 'orange-status'
                  : 'green-status'}"
        ></div>

        <div class="field-header">
            <span class="field-text">{title}</span>
            {#if lastUpdated}
                <span class="timestamp">
                    {rosTimeToFormattedTime(
                        lastUpdated.sec,
                        lastUpdated.nanosec,
                    )}
                </span>
            {/if}
        </div>

        <button on:click={toggleExpand} class="toggle-button">
            {isExpanded ? "▲" : "▼"}
        </button>
    </div>

    {#if isExpanded}
        <div
            in:slide={{ duration: 333 }}
            out:slide={{ duration: 333 }}
            class="telemetry-data"
        >
            <slot {telemetryData}></slot>
        </div>
    {/if}
</div>

<style>
    .field {
        display: flex;
        flex-direction: column;
        width: 100%;
        text-align: left;
        box-sizing: border-box;
    }

    .field-top-row {
        display: flex;
        width: 100%;
        align-items: center;
    }

    .field-header {
        display: flex;
        flex-direction: row;
        align-items: center;
        flex-grow: 1;
        justify-content: space-between;
        margin-right: 0.75rem;
    }

    .field-text {
        font-weight: bold;
    }

    .timestamp {
        color: rgba(204, 204, 220, 0.65);
        font-size: 0.8em;
        margin-left: 8px;
    }

    .toggle-button {
        cursor: pointer;
        background: none;
        border: 1px solid rgba(204, 204, 220, 0.5);
        border-radius: 0.5rem;
        /* margin-right: 1rem; */
    }

    .telemetry-data {
        font-size: 0.95rem;
        padding-left: 0.5rem;
        width: 100%;
        box-sizing: border-box;
    }

    /* Status indicators */
    .status-indicator {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        margin-right: 10px;
    }
</style>
