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

        if (signalPath) animatePath(signalPath, index);

        observer.disconnect();
    });
    observer.observe(svgContainer, { childList: true, subtree: true });
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
    const animationDuration = 333;

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

    // Return cleanup function
    return () => {
        layoutObserver.disconnect();
        resizeObserver.disconnect();
        if (animationFrame) {
            cancelAnimationFrame(animationFrame);
        }
    };
}