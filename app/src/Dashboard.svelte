<script>
  // @ts-nocheck
  import { onMount } from "svelte";
  import SVG from "../public/images/gs3.svg";
  import Field from "./Field.svelte";
  import { animatePath } from "./lib/Utils.svelte";
   
  const host = process.env.IP_ADDRESS;
  console.log(host);

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

    // x="-23.32" y="448.19" width="142.89" height="37.09"
    // x="29.58" y="395.22" width="37.07" height="142.89" />
    const maxHeight = 142.89;
    const maxY = 395.22; // The initial Y position when full

    const svg = document.querySelector("svg");

    switch (frame_id) {
      case "LoadCells":
        const { load_cell_1, load_cell_2 } = telemetryData;
        let newHeight = ((load_cell_1 + load_cell_2) / 200) * maxHeight;
        let newY = maxY - (maxHeight + newHeight);

        let rect = svg.getElementById("oxidizer_liquid");
        rect.setAttribute("height", Math.min(newHeight, 142.89));
        rect.style.fill = "url(#gradient_blue)";
        break;

      case "ValveServos":
        const { servo1_position, servo2_position } = telemetryData;
        const valve1 = svg.getElementById("valve1");
        const valve2 = svg.getElementById("valve2");

        if (servo1_position > 50) valve1.style.stroke = "url(#gradient_green)";
        else valve1.style.stroke = "url(#gradient_red)";

        if (servo2_position > 50) valve2.style.stroke = "url(#gradient_green)";
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

  onMount( async () => {
    fetchSVG();
    fetchConfig();
    observeSVGRender();
  });
</script>

<div id="svg-container">
  {@html svgContent}
</div>

<div id="rocket-info">
  <h3>ROCKET</h3>
  {#each rocket_topics as topic}
    <Field
      class_name="rocket"
      on:telemetryChange={handleTelemetryChange}
      {topic}
    />
  {/each}
</div>

<div id="gs-info">
  <h3>GROUND SEGMENT</h3>
  {#each gs_topics as topic}
    <Field
      class_name="gs"
      on:telemetryChange={handleTelemetryChange}
      {topic}
    />
  {/each}
</div>

<style>
  #svg-container {
    position: absolute;
    top: 15vh;
    left: 27vw;
    width: 60vw;
    /* margin-top: 5%; */
  }

  #rocket-info {
    position: absolute;
    top: 15vh;
    left: 2vw;
    width: 24vw;
    height: 70vh;
    border-radius: 1vw;
    border: 1px solid #eee;
    background-color: #161616;
    overflow: auto;
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
  }

  #gs-info {
    position: absolute;
    top: 15vh;
    left: 68vw;
    height: 80vh;
    width: 30vw;
    border-radius: 1vw;
    border: 1px solid #eee;
    background-color: #161616;
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
    overflow-y: scroll;
  }

  @keyframes dash {
    to {
      stroke-dasharray: 1000;
      opacity: 1;
    }
  }
</style>
