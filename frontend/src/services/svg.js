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

export function observeLines(lines) {
    if (!lines || !lines.length) return;

    // Main observers for layout changes
    const layoutObserver = new MutationObserver((mutations) => {
        const isTelemetryToggle = mutations.some(
            (mutation) =>
                mutation.target.classList &&
                mutation.target.classList.contains("telemetry-data"),
        );

        if (isTelemetryToggle) {
            animateLineUpdates();
        } else {
            scheduleUpdate();
        }
    });

    const resizeObserver = new ResizeObserver(() => {
        scheduleUpdate();
    });
    
    let animationInProgress = false;
    let animationFrame;
    let startTime;
    const animationDuration = 400;

    function animateLineUpdates() {
        if (animationInProgress) return;

        animationInProgress = true;

        startTime = performance.now();

        function animate(currentTime) {
            const elapsed = currentTime - startTime;
            const progress = Math.min(elapsed / animationDuration, 1);

            updateLinePositions();

            if (progress < 1) {
                animationFrame = requestAnimationFrame(animate);
            } else {
                animationInProgress = false;
                updateLinePositions();
            }
        }

        animationFrame = requestAnimationFrame(animate);
    }

    let updateScheduled = false;

    function scheduleUpdate() {
        if (updateScheduled) return;
        updateScheduled = true;

        setTimeout(() => {
            updateLinePositions();
            updateScheduled = false;
        }, 100);
    }

    function updateLinePositions() {
        lines.forEach((line) => {
            try {
                line.position();
            } catch (e) {
                console.warn("Error updating line position:", e);
            }
        });
    }

    // Observe changes in telemetry data
    layoutObserver.observe(document.body, {
        subtree: true,
        attributes: true,
        attributeFilter: ["class"],
    });

    // Observe resize events
    const svgContainer = document.getElementById("svg-container");
    if (svgContainer) {
        resizeObserver.observe(svgContainer);
    }

    const rocketInfo = document.getElementById("rocket-info");
    
    // When the user clicks anywhere in the sidebar, start the loop
    const triggerAnimation = () => {
        animateLineUpdates();
    };

    if (rocketInfo) {
        rocketInfo.addEventListener("click", triggerAnimation);
    }

    return () => {
        layoutObserver.disconnect();
        resizeObserver.disconnect();
       if (rocketInfo) {
            rocketInfo.removeEventListener("click", triggerAnimation);
        }
        if (animationFrame) {
            cancelAnimationFrame(animationFrame);
        }
    };
}

  export function createLines(lines) {
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
  }
