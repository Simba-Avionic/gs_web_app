<script>
  // @ts-nocheck
  import { onMount } from "svelte";
  import SVG from "./assets/gs3.svg";
  import Field from "./Field.svelte";
  import { animatePath } from "./Utils.svelte";

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
        const radio = document.querySelectorAll("#_433_signal path");

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

  async function fetchConfig() {
    const response = await fetch("http://localhost:8000/config");
    const data = await response.json();
    topics = data.topics;

    rocket_topics = topics.filter((topic) => topic.place === "rocket");
    gs_topics = topics.filter((topic) => topic.place === "gs");

    console.log(topics);
  }

  onMount(() => {
    fetchConfig();
    let index = 0;
    animatePath(document.querySelectorAll("#_433_signal path"), index);
    animatePath(document.querySelectorAll("#radiolinia_signal_tx path"), index);
  });
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<div id="svg-container">
  <SVG width="60vw" />

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
</div>

<style>
  #svg-container {
    position: relative;
    width: 70vw;
    margin-top: 5%;
  }

  #rocket-info {
    position: absolute;
    top: 5%;
    left: -18%;
    width: 24%;
    height: 80%;
    border-radius: 1vw;
    border: 1px solid #eee;
    background-color: #242424;
    overflow: auto;
    display: flex;
    flex-direction: column;
    justify-content: flex-start;
  }

  #gs-info {
    position: absolute;
    top: 5%;
    left: 70%;
    width: 40%;
    height: 95%;
    border-radius: 1vw;
    border: 1px solid #eee;
    background-color: #242424;
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
