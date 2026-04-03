<script>
  import { onMount, onDestroy, tick } from "svelte";
  import RocketField from "../components/fields/RocketField.svelte";
  import GaugeField from "../components/fields/GaugeField.svelte";
  import { subscribeToTopic } from "../services/websockets.js";
  import { fetchConfig } from "../services/api.js";

  import {
    fetchSVG,
    createLine,
    observeLines,
  } from "../services/svg.js";

  let svgContent = "";
  let cleanup;

  let lines = [];
  let topics = [];
  let statusItems = [];
  let svgItems = [];
  let rocketGroups = {};

  function createLines() {
    lines.push(createLine("tank_oxidizer", "#rocket-info #tank_sensors"));
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

    /* 
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
    */
  }

  // TODO: Try to remove hardcoded if statements and so on...
  // This is very specific to the current SVG and telemetry config, (in other words - shitty function)
  // but ideally we would want a more flexible system that can adapt to different SVGs and telemetry configs without needing code changes
  function handleTelemetryChange(data, msg_type) {
    const telemetryData = data;
    if (!telemetryData) return;

    const config = svgItems.find((item) => item.msg_type === msg_type);
    if (!config) return;

    const svg = document.getElementById("simba");
    if (!svg) return;

    config.controls.forEach((element) => {
      const fieldName = element.val_name; // e.g. "valve_feed_oxidizer"
      const svgId = element.controls; // e.g. "valve1"
      const value = telemetryData[fieldName];
      
      // @ts-ignore
      const svgElem = svg.getElementById(svgId);

      if (svgId == "oxidizer_liquid") {
        // @ts-ignore
        const OXTank = svg.getElementById("tank_oxidizer");

        const visualMaxHeight = OXTank.getBBox().height;

        const dataMax = config.controls[0].range[1];
        let newHeight = (value / dataMax) * visualMaxHeight;

        svgElem.setAttribute(
          "height",
          Math.min(Math.max(newHeight, 0), visualMaxHeight),
        );
        svgElem.style.fill = "url(#gradient_blue)";
      } else if (svgId == "valve_feed_oxidizer_txt") {
        let label = "";
        const n = Number(value);
        if (!Number.isNaN(n)) {
          if (n <= 5) label = `CLOSED ${Math.round(n)}%`;
          else if (n >= 5) label = `OPENED ${Math.round(n)}%`;
          else label = `${Math.round(n)}%`;
        } else {
          label = value !== undefined && value !== null ? String(value) : "";
        }
        // @ts-ignore
        const txtEl = svg.getElementById("valve_feed_oxidizer_txt");
        if (txtEl) {
          txtEl.textContent = label;
        }
      } else if (svgId == "valve_vent_oxidizer_txt") {
        let label = "";
        const n = Number(value);
        if (!Number.isNaN(n)) {
          if (n == 0) label = `CLOSED`;
          else if (n == 1) label = `OPENED`;
        } else {
          label = value !== undefined && value !== null ? String(value) : "";
        }
        // @ts-ignore
        const txtEl = svg.getElementById("valve_vent_oxidizer_txt");
        if (txtEl) {
          txtEl.textContent = label;
        }
      }
    });
  }

  onMount(async () => {
    svgContent = await fetchSVG("/images/r7.svg");
    topics = await fetchConfig();

    let tempGroups = {};

    topics.forEach((topic) => {
      topic.msg_fields.forEach((field) => {

        if (field.rocket_display) {
          const group = field.rocket_display.group;
          if (!tempGroups[group]) tempGroups[group] = [];

          tempGroups[group].push({
            topic: topic.topic_name,
            fieldName: field.val_name,
            label: field.rocket_display.label,
            format: field.rocket_display.format,
            unit: field.rocket_display.unit || field.unit || "",
            enum: field.enum || null,
          });
        }

        if (field.rocket_display_bits) {
          field.rocket_display_bits.forEach((bitDef) => {
            const group = bitDef.group;
            if (!tempGroups[group]) tempGroups[group] = [];

            tempGroups[group].push({
              topic: topic.topic_name,
              fieldName: field.val_name,
              label: bitDef.label,
              mask: bitDef.mask,
              onLabel: bitDef.on,
              offLabel: bitDef.off,
              unit: "",
            });
          });
        }
      });
    });

    // TRIGGER SVELTE REACTIVITY HERE
    rocketGroups = tempGroups;

    statusItems = topics.flatMap((topic) =>
      topic.msg_fields
        .filter((f) => ["status", "value"].includes(f.display))
        .map((f) => ({
          id: `${topic.topic_name}_${f.val_name}`,
          topic: topic.topic_name,
          field: f.val_name,
          title: f.alt_name ? f.alt_name : f.val_name.replace(/_/g, " "),
          type: f.display,
          unit: f.unit ?? null,
          range: f.range ?? null,
          value: null,
          enum: f.enum ?? null,
        })),
    );

    svgItems = topics
      .filter((t) => t.msg_fields.some((f) => "controls" in f))
      .map((t) => ({
        topic_name: t.topic_name,
        msg_type: t.msg_type,
        // normalize each "controls" to an array and keep the field config
        controls: t.msg_fields
          .filter((f) => "controls" in f)
          .map((f) => ({
            ...f,
            controls: Array.isArray(f.controls) ? f.controls : [f.controls],
          })),
      }));

    svgItems.forEach((topic) => {
      const unsubscribe = subscribeToTopic(
        window.location.host,
        topic.topic_name,
        (data) => {
          handleTelemetryChange(data, topic.msg_type);
        },
      );
    });

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
  <div id="status-container">
    {#each statusItems as item (item.id)}
        <GaugeField {item} />
    {/each}
  </div>

  <div id="svg-container">
    {@html svgContent}
  </div>

  <div id="rocket-info">
    {#if Object.keys(rocketGroups).length > 0}
      <div class="rocket-item" id="recovery">
        <RocketField
          title="Recovery"
          fieldConfigs={rocketGroups["Recovery"] || []}
        />
      </div>
      <div class="rocket-item" id="avionics">
        <RocketField
          title="Avionics"
          fieldConfigs={rocketGroups["Avionics"] || []}
        />
      </div>
      <div class="rocket-item" id="tank_sensors">
        <RocketField
          title="Tank Sensors"
          fieldConfigs={rocketGroups["Tank Sensors"] || []}
        />
      </div>
      <div class="rocket-item" id="tank_actuators">
        <RocketField
          title="Tank Actuators"
          fieldConfigs={rocketGroups["Tank Actuators"] || []}
        />
      </div>
      <div class="rocket-item" id="engine">
        <RocketField
          title="Engine"
          fieldConfigs={rocketGroups["Engine"] || []}
        />
      </div>
    {/if}
  </div>
</div>

<style>
  #layout-container {
    display: flex;
    justify-content: space-between;
    position: relative;
    padding: 1rem;
    margin-top: var(--navbar-height);
    height: calc(100vh - var(--navbar-height));
    box-sizing: border-box;
    z-index: 1;
  }

  #svg-container {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    z-index: -1;
    pointer-events: none;
    user-select: none;
    display: flex;
    justify-content: center;
    align-items: center;
    overflow: hidden;
  }

  #svg-container :global(svg) {
    width: 100%;
    height: auto;
    max-height: 100%;
  }

  #status-container :global(svg) {
    width: 100%;
    height: 100%;
  }

  #status-container {
    padding-top: 1rem;
    display: grid;
    grid-template-columns: repeat(3, minmax(140px, 1fr));
    grid-template-rows: repeat(3, 1fr);
    width: 40%; /* Decide exactly how much screen width the gauges get */
    z-index: 10;
  }

  #rocket-info {
    display: flex;
    flex-direction: column;
    gap: 1rem;
    padding: 0 1rem;
    border-radius: 0.75rem;
    max-height: 100%;
    width: 30%;
    z-index: 10;
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

  @media (min-width: 1280px) {
    #status-container {
      gap: 0.5rem;
    }
  }

  @media (min-width: 1920px) {
    #rocket-info {
      gap: 1rem;
    }
    #status-container {
      gap: 1rem;
      padding: 1rem;
    }
  }

  @media (max-width: 1280px) {
    #rocket-info {
      gap: 0.8rem;
      padding: 0.5rem;
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
