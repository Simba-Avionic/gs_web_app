<script>
  import { onMount, onDestroy, tick } from "svelte";
  import RocketField from "./lib/RocketField.svelte";
  import TelemetryField from "./lib/TelemetryField.svelte";
  import {
    fetchSVG,
    createLine,
    observeSVGRender,
    fetchConfig,
    observeLines,
  } from "./lib/Utils.svelte";

  export let host;

  // TODO: get those values directly from SVG
  const maxHeight = 142.89;

  let svgContent = "";
  let cleanup;

  let lines = [];
  let topics = [];

  function createLines() {
    lines.push(createLine("tank", "#rocket-info #tank_pressure"));
    lines.push(createLine("tank", "#rocket-info #tank_temperature"));
    lines.push(createLine("tank", "#rocket-info #tank_actuators"));
    lines.push(createLine("engine", "#rocket-info #engine"));
    lines.push(createLine("avionics", "#rocket-info #avionics"));
    lines.push(createLine("recovery", "#rocket-info #recovery"));

    const defaultOptions = {
      color: "#ccccdc",
      size: 1,
      hide: true,
      startPlug: "behind",
      endPlug: "behind",
      showEffectName: "draw",
    };

    lines.push(
      createLine("oxidizer_tank", "#valve1", {
        ...defaultOptions,
        path: "grid",
        startSocket: "top",
        endSocket: "left",
        startSocketGravity: [0, 0],
        endSocketGravity: [0, 0],
        dash: {
          animation: true,
          len: 5,
          gap: 3,
        },
      }),
    );

    lines.push(
      createLine("oxidizer_tank", "#valve2", {
        ...defaultOptions,
        path: "grid",
        startSocket: "top",
        endSocket: "top",
        startSocketGravity: [0, 0],
        endSocketGravity: [0, 0],
        dash: {
          animation: true,
          len: 5,
          gap: 3,
        },
      }),
    );

    lines.push(
      createLine("valve1", "#hatch", {
        ...defaultOptions,
        path: "grid",
        startSocket: "right",
        endSocket: "left",
        startSocketGravity: [25, 0],
        endSocketGravity: [0, 0],
        dash: {
          animation: true,
          len: 5,
          gap: 3,
        },
      }),
    );
  }

  function handleTelemetryChange(event) {
    
    const telemetryData = event.detail.data;

    if (telemetryData == null) {
      return;
    }

    const { header } = telemetryData;
    const { frame_id } = header;

    // const {hasNonOkStatus} = telemetryData;

    const svg = document.querySelector("svg");

    switch (frame_id) {
      case "LoadCells":
        const { combined_fuel_kg } = telemetryData;
        let newHeight = (combined_fuel_kg / 20) * maxHeight;

        let rect = svg.getElementById("oxidizer_liquid");
        rect.setAttribute(
          "height",
          // @ts-ignore

          Math.min(Math.max(newHeight, 0), maxHeight),
        );
        // @ts-ignore

        rect.style.fill = "url(#gradient_blue)";
        break;

      case "ValveServos":
        const { valve_feed_position, valve_vent_position } = telemetryData;
        const valve1 = svg.getElementById("valve1");
        const valve2 = svg.getElementById("valve2");

        if (valve_feed_position > 50)
          // @ts-ignore
          valve1.style.stroke = "url(#gradient_green)";
        // @ts-ignore
        else valve1.style.stroke = "url(#gradient_red)";

        if (valve_vent_position > 50)
          // @ts-ignore
          valve2.style.stroke = "url(#gradient_green)";
        // @ts-ignore
        else valve2.style.stroke = "url(#gradient_red)";
        break;

      case "Telemetry433":
        const { noise } = telemetryData;
        const radio = svg.querySelectorAll("#_433_signal path");

        if (noise < -30) {
          radio.forEach((path) => {
            // @ts-ignore
            path.style.stroke = "url(#gradient_green)";
          });
        } else if (noise < -15) {
          radio.forEach((path) => {
            // @ts-ignore
            path.style.stroke = "url(#gradient_orange)";
          });
        } else {
          radio.forEach((path) => {
            // @ts-ignore
            path.style.stroke = "url(#gradient_red)";
          });
        }
        break;
    }

    // if (hasNonOkStatus !== undefined && hasNonOkStatus !== null) {
    //   const statusElement = svg.querySelector("#status");
    //   if (statusElement) {
    //     statusElement.style.fill = "url(#gradient_red)";
    //   }
    // } else {
    //   const statusElement = svg.querySelector("#status");
    //   if (statusElement) {
    //     statusElement.style.fill = "url(#gradient_green)";
    //   }
    // }

  }

  onMount(async () => {
    svgContent = await fetchSVG("/images/gs4.svg");
    topics = await fetchConfig(host);
    observeSVGRender();
    await tick();
    createLines();
    cleanup = observeLines(lines);
  });

  onDestroy(() => {
    lines.forEach((line) => {
      line.remove();
    });
    if (cleanup) cleanup();
  });
</script>

<div id="layout-container">
  <div id="svg-container">
    {@html svgContent}
  </div>

  <div id="rocket-info">
    <div class="rocket-item" id="recovery">
      <RocketField {host} title="Recovery" />
    </div>
    <!-- <div class="rocket-item" id="payload">Payload</div> -->
    <div class="rocket-item" id="avionics">
      <RocketField {host} title="Avionics" />
    </div>
    <div class="rocket-item" id="tank_pressure">
      <RocketField {host} title="Tank Pressure" />
    </div>
    <div class="rocket-item" id="tank_temperature">
      <RocketField {host} title="Tank Temperature" />
    </div>
    <div class="rocket-item" id="tank_actuators">
      <RocketField {host} title="Tank Actuators" />
    </div>
    <div class="rocket-item" id="engine">
      <RocketField {host} title="Engine" />
    </div>
  </div>

  <div id="telemetry-info">
    <div id="telemetry-title">
      <h3>TELEMETRY</h3>
    </div>
    <div class="fields-container">
      {#each topics as topic (topic.id)}
        <TelemetryField
          on:telemetryChange={handleTelemetryChange}
          {topic}
          {host}
        />
      {/each}
    </div>
  </div>
</div>

<style>
  #layout-container {
    display: grid;
    grid-template-columns: 25% 30% 45%;
    grid-template-rows: auto 1fr;
    grid-template-areas:
      "svg rocket gs"
      "svg empty gs";
    gap: 1rem;
    padding: 2rem;
    padding-right: 4rem;
    margin-top: var(--navbar-height);
    height: calc(100vh - var(--navbar-height));
    box-sizing: border-box;
  }

  #svg-container {
    grid-area: svg;
    display: flex;
    justify-content: left;
    align-items: center;
    max-width: 100%;
    height: 100%;
  }

  #svg-container :global(svg) {
    max-height: 85vh;
    width: auto;
    height: auto;
  }

  #rocket-info {
    grid-area: rocket;
    display: flex;
    flex-direction: column;
    gap: 1rem;
    padding: 0 1rem;
    border-radius: 0.75rem;
    max-height: 100%;
    /* overflow-y: auto; */
  }

  #telemetry-info {
    grid-area: gs;
    border-radius: 0.75rem;
    border: 1px solid rgba(204, 204, 220, 0.5);
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
    overflow-y: auto;
    max-height: calc(100vh - var(--navbar-height));
    text-align: center;
    margin-bottom: 1rem;
    background-color: #181b1f;
  }

  #telemetry-info .fields-container {
    display: grid;
    grid-template-columns: 1fr 1fr;
    width: 100%;
    position: relative;
  }

  .rocket-item {
    border: 1px solid rgba(204, 204, 220, 0.5);
    background-color: #181b1f;
    border-radius: 0.75rem;
    padding: 1rem;
  }

  @keyframes dash {
    to {
      stroke-dasharray: 1000;
      opacity: 1;
    }
  }

  @media (max-width: 1280px) {
    #telemetry-info .fields-container {
      grid-template-columns: 1fr; /* Change to one column */
    }

    #telemetry-info .fields-container::after {
      display: none;
    }

    #layout-container {
      grid-template-columns: 25% 35% 40%; /* Optional: adjust main layout proportions */
    }
  }

  @media (min-width: 1920px) {
    #rocket-info {
      gap: 1rem;
    }

    #layout-container {
      padding: 4rem;
      padding-right: 6rem;
    }
  }
</style>
