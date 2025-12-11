<script>
  import { onMount, onDestroy, tick } from "svelte";
  import RocketField from "./lib/RocketField.svelte";
  import GaugeField from "./lib/GaugeField.svelte";
  import StatusField from "./lib/StatusField.svelte";
  import { subscribeToTopic } from "./js/ws_manager.js";

  import {
    fetchSVG,
    createLine,
    observeSVGRender,
    fetchConfig,
    observeLines,
  } from "./lib/Utils.svelte";

  export let host;

  let svgContent = "";
  let cleanup;

  let lines = [];
  let topics = [];
  let statusItems = [];
  let svgItems = [];

  function createLines() {
    lines.push(createLine("tank_oxidizer", "#rocket-info #tank_pressure"));
    lines.push(createLine("tank_oxidizer", "#rocket-info #tank_temperature"));
    lines.push(createLine("tank_oxidizer", "#rocket-info #tank_actuators"));
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

  function handleTelemetryChange(data, msg_type) {
    const telemetryData = data;
    if (!telemetryData) return;

    const config = svgItems.find((item) => item.msg_type === msg_type);
    if (!config) return; // ignore unknown messages

    const svg = document.getElementById("simba");
    if (!svg) return; // exit if SVG not yet in DOM

    // const tanking_group = document.getElementById("tanking_group");

    config.controls.forEach((field, idx) => {
      const fieldName = field.val_name; // e.g. "valve_feed_oxidizer"
      const svgId = field.controls; // e.g. "valve1"
      const value = telemetryData[fieldName];

      // console.log(svgId);

      const svgElem = svg.getElementById(svgId);
      if (!svgElem) return;


      // if (svgId.includes("liquid")) {
      //   console.log(value);
      // }

      // if (field.type !== "bool") {
      //   if (value > 50) {
      //     svgElem.style.stroke = "url(#gradient_green)";
      //   } else {
      //     svgElem.style.stroke = "url(#gradient_red)";
      //   }
      // } else {
      //   if (value) {
      //     svgElem.style.stroke = "url(#gradient_green)";
      //   } else {
      //     svgElem.style.stroke = "url(#gradient_red)";
      //   }
      // }

      // let maxHeight = 102;
      // let newHeight = (value / 10) * maxHeight;
      // // let rect = svg.getElementById("oxidizer_liquid");
      // svgElem.setAttribute(
      //   "height",
      //   Math.min(Math.max(newHeight, 0), maxHeight),
      // );
      // svgElem.style.fill = "url(#gradient_blue)";

      // // let maxHeight = 102;
      // // let newHeight = (value / 20) * maxHeight;
      // // let rect2 = svg.getElementById("pressurizer_liquid");
      // svgElem.setAttribute(
      //   "height",
      //   Math.min(Math.max(newHeight, 0), maxHeight),
      // );
      // svgElem.style.fill = "url(#gradient_blue)";
    });
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
          title: f.alt_name ? f.alt_name : f.val_name.replace(/_/g, " "),
          type: f.display,
          unit: f.unit ?? null,
          range: f.range ?? null,
          value: null,
        })),
    );

    svgItems = topics
      .filter((t) => t.msg_fields.some((f) => "controls" in f))
      .map((t) => ({
        topic_name: t.topic_name,
        msg_type: t.msg_type,
        controls: t.msg_fields.filter((f) => "controls" in f),
      }));

    svgItems.forEach((topic) => {
      const unsubscribe = subscribeToTopic(host, topic.topic_name, (data) => {
        handleTelemetryChange(data, topic.msg_type);
      });
    });

    await tick();
    createLines();
    cleanup = observeLines(lines);
  });

  onDestroy(() => {
    lines.forEach((line) => {
      line.remove();
    });
    // Object.values(wsConnections).forEach((ws) => ws.close());
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
    grid-template-columns: repeat(3, minmax(130px, 1fr));
    grid-template-rows: repeat(3, 1fr);
    gap: 0.5rem;
  }

  #status-container > * {
    width: 100%;
    height: 100%;
    display: flex;
    justify-content: center;
    align-items: center;
  }

  #status-container :global(svg) {
    width: 100%;
    height: 100%;
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
