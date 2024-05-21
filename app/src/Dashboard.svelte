<script>
  // @ts-nocheck
  import { onMount } from "svelte";
  import SVG from "./assets/gs3.svg";
  import GsField from "./GSField.svelte";
  import RocketField from "./RocketField.svelte"

  let topics = [];
  let gs_topics = [];
  let rocket_topics = [];

  function changeColor() {
    const svg = document.querySelector("svg");
    const rects = svg.querySelectorAll("rect");
    const paths = svg.querySelectorAll("path");
    paths.forEach((path) => {
      // console.log(path.id);

      if (
        path.id === "oxidizer" ||
        path.id === "fuel" ||
        path.id === "skrzynka_tankowanie" ||
        path.id === "skrzynka_mc" ||
        path.id === "tenso1" ||
        path.id === "tenso2" ||
        path.id === "computer" ||
        path.id === "parachute" ||
        path.id === "payload"
      ) {
        path.style.fill = "url(#gradient_green)";
      } else if (
        path.id === "valve1" ||
        path.id === "valve2" ||
        path.id === "hatch"
      ) {
        path.style.stroke = "url(#gradient_red)";
      }
    });

    rects.forEach((path) => {
      if (
        path.id === "skrzynka_tankowanie" ||
        path.id === "skrzynka_mc" ||
        path.id === "tenso1" ||
        path.id === "tenso2"
      ) {
        path.style.fill = "url(#gradient_green)";
      } else if (
        path.id === "hatch"
      ) {
        path.style.fill = "url(#gradient_red)";
      }
    });
  }

  function animatePath(paths, index) {
    paths[index].animate([{ opacity: "0" }, { opacity: "1" }], {
      duration: 1000,
      easing: "ease-out",
      fill: "forwards",
    }).onfinish = () => {
      index++;
      if (index >= paths.length) {
        index = 0;
      }
      animatePath(paths, index);
    };
  }

  onMount(() => {
    let index = 0;
    const paths1 = document.querySelectorAll("#_433_signal path");
    const paths2 = document.querySelectorAll("#radiolinia_signal_rx path");
    const paths3 = document.querySelectorAll("#radiolinia_signal_tx path");

    paths1.forEach((path) => {
      path.style.stroke = "url(#gradient_red)";
    });
    paths2.forEach((path) => {
      path.style.stroke = "url(#gradient_green)";
    });
    paths3.forEach((path) => {
      path.style.stroke = "url(#gradient_orange)";
    });

    animatePath(paths1, index);
    animatePath(paths2, index);
    animatePath(paths3, index);
  });

  async function fetchData() {
    const response = await fetch("http://localhost:8000/config");
    const data = await response.json();
    topics = data.topics;

    rocket_topics = topics.filter(topic => topic.place === 'rocket');
    gs_topics = topics.filter(topic => topic.place === 'gs');

    console.log(topics);
  }
  onMount(fetchData);
</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<div on:click={changeColor} id="svg-container">
  <SVG width="60vw" />

  <div id="rocket-info">
    <h3>ROCKET</h3>
    {#each rocket_topics as topic}
      <RocketField {topic}/>
    {/each}
  </div>

  <div id="gs-info">
    <h3>GROUND SEGMENT</h3>
    {#each gs_topics as topic}
      <GsField {topic}/>
    {/each}
  </div>
</div>

<style>
  #svg-container {
    position: relative;
    width: 70vw;
  }

  #rocket-info {
    position: absolute;
    top: 5%;
    left: -15%;
    width: 20%;
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
    left: 60%;
    width: 50%;
    height: 70%;
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
