<script>
  // @ts-nocheck
  import { onMount } from "svelte";
  import SVG from "../public/images/gs3.svg";
  import Field from "./Field.svelte";
  import { animatePath } from "./lib/Utils.svelte";

  export let host;

  // TODO: get those values directly from SVG
  const maxHeight = 142.89;
  const maxY = 395.22; // The initial Y position when full

  let svgContent = "";

  let topics = [];
  let gs_topics = [];
  let rocket_topics = [];

  function handleTelemetryChange(event) {
    const telemetryData = event.detail;

    if (telemetryData == null) {
      return;
    }

    const { header } = telemetryData;
    const { frame_id } = header;

    const svg = document.querySelector("svg");

    switch (frame_id) {
      case "LoadCells":
        const { combined_fuel_kg } = telemetryData;
        let newHeight = (combined_fuel_kg / 100) * maxHeight;
        let newY = maxY - (maxHeight + newHeight);

        let rect = svg.getElementById("oxidizer_liquid");
        rect.setAttribute("height", Math.min(Math.max(newHeight, 0), 142.89));
        rect.style.fill = "url(#gradient_blue)";
        break;

      case "ValveServos":
        const { valve_feed_position, valve_vent_position } = telemetryData;
        const valve1 = svg.getElementById("valve1");
        const valve2 = svg.getElementById("valve2");

        if (valve_feed_position > 50)
          valve1.style.stroke = "url(#gradient_green)";
        else valve1.style.stroke = "url(#gradient_red)";

        if (valve_vent_position > 50)
          valve2.style.stroke = "url(#gradient_green)";
        else valve2.style.stroke = "url(#gradient_red)";

        break;

      case "ValveSensors":
        break;

      case "ControlPanelSwitches":
        break;

      case "TankingCmds":
        break;

      case "Telemetry433":
        const { noise } = telemetryData;
        const radio = svg.querySelectorAll("#_433_signal path");

        if (noise < -30) {
          radio.forEach((path) => {
            path.style.stroke = "url(#gradient_green)";
          });
        } else if (noise < -15) {
          radio.forEach((path) => {
            path.style.stroke = "url(#gradient_orange)";
          });
        } else {
          radio.forEach((path) => {
            path.style.stroke = "url(#gradient_red)";
          });
        }
        break;

      case "RocketTelemetry":
        break;

      case "RocketStatus":
        break;
    }
  }

  async function fetchSVG() {
    const response = await fetch("../public/images/gs3.svg");
    svgContent = await response.text();
  }

  async function fetchConfig() {
    const response = await fetch(`http://${host}:8000/config`);
    const data = await response.json();
    topics = data.topics;

    rocket_topics = topics.filter((topic) => topic.place === "rocket");
    gs_topics = topics.filter((topic) => topic.place === "gs");

    console.log(topics);
  }

  function observeSVGRender() {
    const svgContainer = document.getElementById("svg-container");
    const observer = new MutationObserver(() => {
      let index = 0;

      const signalPath = document.querySelectorAll("#_433_signal path");
      const txPath = document.querySelectorAll("#radiolinia_signal_tx path");

      if (signalPath) animatePath(signalPath, index);
      if (txPath) animatePath(txPath, index);

      observer.disconnect();
    });
    observer.observe(svgContainer, { childList: true, subtree: true });
  }

  onMount(async () => {
    fetchSVG();
    fetchConfig();
    observeSVGRender();
  });
</script>

<div id="layout-container">
  <div id="rocket-info">
    <h3>ROCKET</h3>
    {#each rocket_topics as topic}
      <Field
        class_name="rocket"
        on:telemetryChange={handleTelemetryChange}
        {topic}
        {host}
      />
    {/each}
  </div>

  <div id="svg-container">
    {@html svgContent}
  </div>

  <div id="gs-info">
    <h3>GROUND SEGMENT</h3>
    {#each gs_topics as topic}
      <Field
        class_name="gs"
        on:telemetryChange={handleTelemetryChange}
        {topic}
        {host}
      />
    {/each}
  </div>
</div>

<style>

#layout-container {
  display: grid;
  grid-template-columns: 2fr 6fr 4fr;
  grid-template-rows: auto;
  gap: 1rem;
  padding: 2rem;
  margin-top: 60px;
  height: calc(100vh - 60px);
  box-sizing: border-box;
  overflow: hidden;
  text-align: center;
}

#rocket-info, #svg-container, #gs-info {
  border-radius: 1rem;
  max-width: 95%;
  flex: 1;
}

#rocket-info, #gs-info {
  border: 1px solid rgba(204, 204, 220, 0.5);
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  overflow-y: auto;
}

#svg-container {
  display: flex;
  justify-content: center;
  align-items: center;
}

@keyframes dash {
  to {
    stroke-dasharray: 1000;
    opacity: 1;
  }
}
</style>
