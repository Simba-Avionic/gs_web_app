<script>
  import { onMount, onDestroy, tick } from "svelte";
  import RocketField from "./lib/RocketField.svelte";
  import GaugeField from "./lib/GaugeField.svelte";
  import StatusField from "./lib/StatusField.svelte";

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
  let statusItems = [];

  function createLines() {
    lines.push(createLine("tank", "#rocket-info #tank_pressure"));
    lines.push(createLine("tank", "#rocket-info #tank_temperature"));
    lines.push(createLine("tank", "#rocket-info #tank_actuators"));
    lines.push(createLine("engine", "#rocket-info #engine"));
    lines.push(createLine("avionics", "#rocket-info #avionics"));
    lines.push(createLine("recovery", "#rocket-info #recovery"));

    const defaultOptions = {
      color: "var(--text-color)",
      size: 1,
      hide: true,
      startPlug: "behind",
      endPlug: "behind",
      showEffectName: "draw",
    };

    lines.push(
      createLine("oxidizer_tank", "#valve_feed_oxidizer", {
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
      createLine("oxidizer_tank", "#valve_vent_oxidizer", {
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
      createLine("valve_feed_oxidizer", "#decoupler_oxidizer", {
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

    lines.push(
      createLine("pressurizer_tank", "#valve_feed_pressurizer", {
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
      createLine("pressurizer_tank", "#valve_vent_pressurizer", {
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
      createLine("valve_feed_pressurizer", "#decoupler_pressurizer", {
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
  }

  onMount(async () => {
    svgContent = await fetchSVG("/images/gs.svg");
    topics = await fetchConfig(host);
    statusItems = topics.flatMap((topic) =>
      topic.msg_fields
        .filter((f) => ["gauge", "status", "value"].includes(f.display))
        .map((f) => ({
          id: `${topic.topic_name}_${f.val_name}`,
          topic: topic.topic_name,
          field: f.val_name,
          title: f.val_name.replace(/_/g, " "), // readable label
          type: f.display, // "gauge" or "status"
          unit: f.unit ?? null,
          range: f.range ?? null,
          value: null,
        })),
    );

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
  <!-- Status, Gauge, Value -->
  <div id="status-container">
    {#each statusItems as item (item.id)}
    {#if item.type === "value"}
      <GaugeField {host} {item} />
    {/if}
    {/each}
    <!-- {#each statusItems as item (item.id)}
      {#if item.type === "status"}
        <StatusField {host} {item} />
      {/if}
    {/each} -->
  </div>

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
</div>

<style>
  #layout-container {
    display: grid;
    grid-template-columns: 35% 30% 35%;
    grid-template-areas: "gs svg rocket";
    /* gap: 1rem; */
    padding: 1rem;
    margin-top: var(--navbar-height);
    height: calc(100vh - var(--navbar-height));
    box-sizing: border-box;
  }

  #status-container {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    grid-template-rows: repeat(3, minmax(110px, 1fr));
    gap: 1rem;
  }


  #svg-container {
    display: flex;
    justify-content: center;
    align-items: center;
    width: 100%;
    height: 100%;
    overflow: hidden;
  }

  #svg-container :global(svg) {
    width: 100%;
    height: auto;
    max-height: 100%;
  }

  #rocket-info {
    grid-area: rocket;
    display: flex;
    flex-direction: column;
    gap: 1rem;
    padding: 0 1rem;
    border-radius: 0.75rem;
    max-height: 100%;
    margin-left: 1.5rem;
  }

  .rocket-item {
    border: 1px solid var(--border-color);
    background-color: var(--snd-bg-color);
    border-radius: 0.75rem;
    padding: 1rem;
  }

  @keyframes dash {
    to {
      stroke-dasharray: 1000;
      opacity: 1;
    }
  }

  @media (min-width: 1920px) {
    #rocket-info {
      gap: 1rem;
    }
      #status-container {
    gap: 3rem;
    padding: 3rem;
  }
  }

  @media (max-width: 1280px) {
    #rocket-info {
      gap: 0.8rem;
      padding: 0.5rem;
    }

    #layout-container {
      grid-template-columns: 40% 30% 30%;
    }

    .rocket-item {
      border: 1px solid var(--border-color);
      background-color: var(--snd-bg-color);
      border-radius: 0.75rem;
      padding: 0.5rem;
      font-size: 0.95rem;
    }
  }
</style>
