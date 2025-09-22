<script context="module">
  export function rosTimeToFormattedTime(iso = false, secs, nsecs) {
    if (typeof secs !== "number" || typeof nsecs !== "number") {
      console.error("Invalid ROS time:", secs, nsecs);
      return "--";
    }
    const ms = secs * 1000 + nsecs / 1e6;
    if (isNaN(ms) || ms <= 0) {
      console.error("Invalid milliseconds:", ms);
      return "--";
    }

    const date = new Date(ms);

    if (iso) {
      return new Date(ms).toISOString();
    }

    const options = {
      year: "numeric",
      month: "2-digit",
      day: "2-digit",
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
      timeZone: "UTC",
    };

    // @ts-ignore
    return date.toLocaleString("en-GB", options);
  }

  export function animatePath(paths, index) {
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

  export async function fetchSVG(svgPath) {
    const response = await fetch(svgPath);
    return await response.text();
  }

  export function createLine(
    start_name,
    end_name,
    options = {
      color: "var(--text-color)",
      size: 1,
      hide: true,
      startPlug: "behind",
      endPlug: "behind",
      showEffectName: "draw",
      path: "fluid",
      startSocket: "right",
      endSocket: "left",
    },
  ) {
    const svgContainer = document.getElementById("svg-container");
    const svg = svgContainer.querySelector("svg");

    let start = svg.getElementById(start_name);
    let end = document.querySelector(end_name);

    let line = new LeaderLine(start, end, options);

    line.show(["draw"]);
    return line;
  }

  export function observeSVGRender() {
    const svgContainer = document.getElementById("svg-container");
    const observer = new MutationObserver(() => {
      let index = 0;

      const signalPath = document.querySelectorAll("#_433_signal path");
      // const txPath = document.querySelectorAll("#radiolinia_signal_tx path");

      if (signalPath) animatePath(signalPath, index);
      // if (txPath) animatePath(txPath, index);

      observer.disconnect();
    });
    observer.observe(svgContainer, { childList: true, subtree: true });
  }

  export function observeLines(lines) {
    if (!lines || !lines.length) return;

    // Main observers for layout changes
    const layoutObserver = new MutationObserver((mutations) => {
      // Check if this is a telemetry-data expansion/collapse
      const isTelemetryToggle = mutations.some(
        (mutation) =>
          mutation.target.classList &&
          mutation.target.classList.contains("telemetry-data"),
      );

      if (isTelemetryToggle) {
        // If it's a telemetry toggle, use the animation timeline
        animateLineUpdates();
      } else {
        // For other changes, use normal debounced update
        scheduleUpdate();
      }
    });

    const resizeObserver = new ResizeObserver(() => {
      scheduleUpdate();
    });

    // Setup animation for line updates
    let animationInProgress = false;
    let animationFrame;
    let startTime;
    const animationDuration = 333; // Match the slide duration

    function animateLineUpdates() {
      if (animationInProgress) return;

      animationInProgress = true;
      startTime = performance.now();

      function updateFrame(timestamp) {
        const elapsed = timestamp - startTime;
        const progress = Math.min(elapsed / animationDuration, 1);

        // Update lines with easing
        lines.forEach((line) => {
          if (line && typeof line.position === "function") {
            line.position();
          }
        });

        if (progress < 1) {
          animationFrame = requestAnimationFrame(updateFrame);
        } else {
          // Final update at the end of animation
          lines.forEach((line) => {
            if (line && typeof line.position === "function") {
              line.position();
            }
          });
          animationInProgress = false;
        }
      }

      animationFrame = requestAnimationFrame(updateFrame);
    }

    // Observer configurations
    const observerConfig = {
      attributes: true,
      childList: true,
      subtree: true,
      attributeFilter: ["style", "class", "width", "height", "transform"],
    };

    // Elements to observe
    const elementsToObserve = [
      document.getElementById("svg-container"),
      document.getElementById("rocket-info"),
    ];

    // Watch rocket items
    document.querySelectorAll(".rocket-item").forEach((item) => {
      layoutObserver.observe(item, observerConfig);
      resizeObserver.observe(item);
    });

    // Watch toggle buttons specifically to catch the click before animation starts
    document.querySelectorAll(".toggle-button").forEach((button) => {
      button.addEventListener("click", () => {
        // Pre-emptively schedule animation updates when toggle is clicked
        setTimeout(animateLineUpdates, 0);
      });
    });

    // Watch main containers
    elementsToObserve.forEach((element) => {
      if (element) {
        layoutObserver.observe(element, observerConfig);
        resizeObserver.observe(element);
      }
    });

    // Add specific observer for telemetry-data elements
    document.querySelectorAll(".telemetry-data").forEach((element) => {
      // This catches the elements already in the DOM
      element.addEventListener("transitionstart", animateLineUpdates);
      element.addEventListener("transitionend", () => {
        // Final update when transition completes
        setTimeout(() => lines.forEach((line) => line.position()), 50);
      });
    });

    // Watch for new telemetry-data elements that might be added
    const telemetryObserver = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.type === "childList" && mutation.addedNodes.length) {
          Array.from(mutation.addedNodes).forEach((node) => {
            if (node.classList && node.classList.contains("telemetry-data")) {
              node.addEventListener("transitionstart", animateLineUpdates);
              node.addEventListener("transitionend", () => {
                setTimeout(() => lines.forEach((line) => line.position()), 50);
              });
            }
          });
        }
      });
    });

    document.querySelectorAll(".field-content").forEach((element) => {
      telemetryObserver.observe(element, { childList: true, subtree: true });
    });

    // Standard window event listeners
    // window.addEventListener("resize", scheduleUpdate);
    // window.addEventListener("scroll", scheduleUpdate);

    // Debounce variables
    let updateTimeout;

    function scheduleUpdate() {
      if (animationInProgress) return;

      clearTimeout(updateTimeout);
      updateTimeout = setTimeout(() => {
        lines.forEach((line) => {
          if (line && typeof line.position === "function") {
            line.position();
          }
        });
      }, 50);
    }

    return function cleanup() {
      layoutObserver.disconnect();
      resizeObserver.disconnect();
      telemetryObserver.disconnect();
      // window.removeEventListener("resize", scheduleUpdate);
      // window.removeEventListener("scroll", scheduleUpdate);
      clearTimeout(updateTimeout);
      cancelAnimationFrame(animationFrame);

      // Remove event listeners
      document.querySelectorAll(".toggle-button").forEach((button) => {
        button.removeEventListener("click", animateLineUpdates);
      });

      document.querySelectorAll(".telemetry-data").forEach((element) => {
        element.removeEventListener("transitionstart", animateLineUpdates);
        element.removeEventListener("transitionend", animateLineUpdates);
      });
    };
  }

  export async function fetchConfig(host, forceRefresh = true) {
    let topics = [];
    if (!forceRefresh) {
      try {
        const savedTopics = localStorage.getItem("topics");

        if (savedTopics) {
          try {
            const parsedTopics = JSON.parse(savedTopics);
            // Add guard clause to check if parsed topics are valid
            if (
              parsedTopics &&
              Array.isArray(parsedTopics) &&
              parsedTopics.length > 0
            ) {
              topics = parsedTopics;
              console.log("Loaded topics from LocalStorage");
            } else {
              console.log(
                "Saved topics were invalid or empty, fetching from server",
              );
              topics = await fetchConfigFromServer(host);
            }
          } catch (parseError) {
            console.error("Error parsing saved topics:", parseError);
            topics = await fetchConfigFromServer(host);
          }
        } else {
          topics = await fetchConfigFromServer(host);
        }
      } catch (storageError) {
        console.error("LocalStorage error:", storageError);
        topics = await fetchConfigFromServer(host);
      }
    } else {
      topics = await fetchConfigFromServer(host);
    }

    // console.log(topics);
    return topics;
  }

  async function fetchConfigFromServer(host) {
    let topics = [];
    try {
      const response = await fetch(`http://${host}/config`);
      const data = await response.json();
      topics = data.topics;

      try {
        // localStorage.setItem("topics", JSON.stringify(topics));
        console.log("Saved topics to LocalStorage");
      } catch (saveError) {
        console.error("Error saving to LocalStorage:", saveError);
      }
    } catch (fetchError) {
      console.error("Error fetching config:", fetchError);
      topics = []; // Fallback to empty array
    }
    return topics;
  }

  export async function fetchMessageDefs(host) {
    let msg_defs = [];
    try {
      const response = await fetch(`http://${host}/config`);
      const data = await response.json();
      msg_defs = data.msg_defs;

      try {
        // localStorage.setItem("topics", JSON.stringify(topics));
        console.log("Saved topics to LocalStorage");
      } catch (saveError) {
        console.error("Error saving to LocalStorage:", saveError);
      }
    } catch (fetchError) {
      console.error("Error fetching config:", fetchError);
      msg_defs = []; // Fallback to empty array
    }
    return msg_defs;
  }

  export function renderField(telemData, field) {
    const value = telemData[field.val_name];

    // Check if the field is an object (like LoadCell)
    if (typeof value === "object" && value !== null) {
      return Object.keys(value)
        .map((key) => {
          return `
          <div class="nested-field">
            <span>${key}:</span>
            <span>${parseFloat(value[key]).toFixed(2)}</span>
          </div>
        `;
        })
        .join("");
    } else {
      if (field.type.includes("float")) {
        return `<span>${parseFloat(value).toFixed(2)}</span>`;
      } else {
        return `<span>${value}</span>`;
      }
    }
  }

  export function getStatusString(statusCode) {
    switch (statusCode) {
      case 0:
        return "OK";
      case 1:
        return "NOT DEFINED";
      case 2:
        return "ERROR";
      case 3:
        return "CONNECTION ERROR";
      case 4:
        return "INITIALIZE ERROR";
      case 5:
        return "BAD VARIABLE SIZE";
      case 6:
        return "INVALID STATE";
      default:
        return "UNKNOWN STATUS";
    }
  }

  export function stripSimbaPrefix(str) {
    if (typeof str !== "string") {
      return String(str);
    }

    return str.replace(/simba_/g, "");
  }
</script>
