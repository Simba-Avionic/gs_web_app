<script>
  // @ts-nocheck

  import SVG from "./assets/gs3.svg";
  import { onMount } from "svelte";

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
        path.id === "tenso2"
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
      } else if (path.id === "hatch") {
        path.style.stroke = "url(#gradient_red)";
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

  let latestNumber = null;

  const evtSource = new EventSource("http://localhost:8000/number");
  evtSource.onmessage = function (event) {
    latestNumber = event.data;
  };

  let canvas;
  let ctx;
  let animationId;

  onMount(() => {
    // Get the canvas element and its context
    canvas = document.getElementById('myCanvas');
    ctx = canvas.getContext('2d');

    // Set initial variables
    let xOffset = 0;

    // Animation loop
    function animate() {
      // Clear the canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Draw the sine wave
      drawSineWave(xOffset);

      // Update xOffset for the next frame
      xOffset += 1;

      // Request the next frame
      animationId = requestAnimationFrame(animate);
    }

    // Function to draw a sine wave
    function drawSineWave(xOffset) {
      ctx.beginPath();
      for (let x = 0; x < canvas.width; x++) {
        const y = 10 * Math.sin((x + xOffset) * 0.05) + canvas.height / 4;
        ctx.lineTo(x, y);
      }
      ctx.strokeStyle = 'lightgreen';
      ctx.stroke();
    }


    // Start the animation
    animate();

    // Cleanup function to stop animation when component is unmounted
    return () => {
      cancelAnimationFrame(animationId);
    };
  });

</script>

<!-- svelte-ignore a11y-click-events-have-key-events -->
<!-- svelte-ignore a11y-no-static-element-interactions -->
<div on:click={changeColor} id="svg-container">
  <SVG width="60vw" />

  <div id="rocket-info">
    <h3>ROCKET</h3>
    <div class="rocket-field">
      <div class="rocket-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="rocket-field-text">Field 1:</span>
      <span class="rocket-field-value">{latestNumber/4.0}</span>
    </div>
    <div class="rocket-field">
      <div class="rocket-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="rocket-field-text">Field 2:</span>
      <span class="rocket-field-value">{latestNumber/2.0}</span>
    </div>
    <div class="rocket-field">
      <div class="rocket-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="rocket-field-text">Field 3:</span>
      <span class="rocket-field-value">{latestNumber/4.0}</span>
    </div>
    <div class="rocket-field">
      <div class="rocket-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="rocket-field-text">Field 4:</span>
      <span class="rocket-field-value">{latestNumber/2.0}</span>
    </div>
    <div class="rocket-field">
      <div class="rocket-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="rocket-field-text">Field 5:</span>
      <span class="rocket-field-value">{latestNumber/4.0}</span>
    </div>
    <div class="rocket-field">
      <div class="rocket-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="rocket-field-text">Field 6:</span>
      <span class="rocket-field-value">{latestNumber/2.0}</span>
    </div>
  </div>

  <div id="gs-info">
    <h3>GROUND SEGMENT</h3>
    <div class="gs-field">
      <div class="gs-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="gs-field-text">Field 2:</span>
      <span class="gs-field-value">{latestNumber}</span>
    </div>
    <div class="gs-field">
      <div class="gs-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="gs-field-text">Field 3:</span>
      <span class="gs-field-value">{latestNumber}</span>
    </div>
    <div class="gs-field">
      <div class="gs-status-indicator {latestNumber != 'None' || null ? 'green-status' : 'red-status'}"></div>
      <span class="gs-field-text">Field 5:</span>
      <span class="gs-field-value">{latestNumber}</span>
      <canvas id="myCanvas" width="100" height="50"></canvas>
    </div>
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
    overflow: auto; /* Add overflow to enable scrolling if needed */
    /* white-space: nowrap; Prevent text from wrappin */
    display: flex; /* Add flex display */
    flex-direction: column; /* Set flex direction to column */
    justify-content: flex-start; /* Align items to the start (left) */
  }

  #gs-info {
    position: absolute;
    top: 5%;
    left: 60%;
    width: 50%;
    height: 60%;
    border-radius: 1vw;
    border: 1px solid #eee;
    background-color: #242424;
    /* overflow: auto;
    white-space: nowrap; */
    display: flex; /* Add flex display */
    flex-direction: column; /* Set flex direction to column */
    justify-content: flex-start; /* Align items to the start (left) */
  }

  @keyframes dash {
    to {
      stroke-dasharray: 1000;
      opacity: 1;
    }
  }

  .rocket-field {
    white-space: nowrap;
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 10px;
    border-bottom: 1px solid #eee; /* Add border to the bottom of each field */
    text-align: left;
  }

  .rocket-field:last-child {
    border-bottom: none; /* Remove border from the last field */
  }

  .rocket-field-text {
    flex: 1;
    color: #fff; /* Text color */
    min-width: 0; /* Allow text to overflow if needed */
    overflow: hidden; /* Hide overflow text */
    white-space: nowrap; /* Prevent text wrapping */
    text-overflow: ellipsis; /* Show ellipsis if text overflows */
  }

  .rocket-status-indicator {
    width: 16px; /* Adjust width as needed */
    height: 16px; /* Adjust height as needed */
    border-radius: 50%;
    margin-right: 10px;
  }

  .rocket-field-value {
    color: whitesmoke; /* Text color */
  }

  .gs-field {
    display: flex;
    align-items: center;
    padding: 12px;
    border-bottom: 1px solid #eee; /* Add border to the bottom of each field */
    text-align: left;
  }

  .gs-field:last-child {
    border-bottom: none; /* Remove border from the last field */
  }

  .gs-field-text {
    flex: 0.2;
    color: #fff; /* Text color */
  }

  .gs-status-indicator {
    width: 16px; /* Adjust width as needed */
    height: 16px; /* Adjust height as needed */
    border-radius: 50%;
    margin-right: 16px;
  }

  .gs-field-value {
    color: whitesmoke; /* Text color */
  }

  .green-status{
    background: linear-gradient(90deg, #7fff7f, #5eff5e, #3dff3d, #1aff1a);
    background-size: 400% 400%;
    animation: gradientAnimation 3s ease infinite;
}

.orange-status {
    background: linear-gradient(45deg, orange, yellow);
    background-size: 200% 200%;
    animation: orangeGradientAnimation 3s ease infinite;
  }

  .red-status {
    background: linear-gradient(45deg, red, pink);
    background-size: 200% 200%;
    animation: redGradientAnimation 3s ease infinite;
  }

@keyframes gradientAnimation {
    0% {
        background-position: 0% 50%;
    }
    50% {
        background-position: 100% 50%;
    }
    100% {
        background-position: 0% 50%;
    }
}

</style>
