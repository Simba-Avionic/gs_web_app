import { cssVar } from "./colors.js"; // your helper for CSS variables

function generateTicks(range) {
    const [min, max] = range;
    if (max - min <= 1) {
        return [min, max];
    }
    const step = (max - min) / 2;
    return [min, min + step, max];
}

export function getPlotLayout(field) {
    const isBool = field.type === "bool";

    return {
        margin: { t: 10, b: 30, l: 0, r: 0 },
        title: "",
        xaxis: {
            title: "Time",
            type: "date",
            tickformat: "%H:%M:%S",
            tickfont: { size: 10 },
            showgrid: true,
            zeroline: false,
            gridcolor: cssVar("--nav-active"),
        },
        yaxis: isBool
            ? {
                title: {
                    text: field.alt_name || field.val_name,
                    standoff: 20,
                    font: { size: 13 },
                },
                tickvals: [0, 1],
                ticktext: ["False", "True"],
                tickfont: { size: 10 },
                autorange: false,
                showgrid: true,
                zeroline: false,
                automargin: true,
                gridcolor: cssVar("--nav-active"),
            }
            : {
                title: {
                    text: `${field.alt_name || field.val_name} ${field.unit ? `(${field.unit})` : ""
                        }`,
                    standoff: 20,
                    font: { size: 13 },
                },
                range: field.range || [0, 100],
                tickvals: generateTicks(field.range || [0, 100]),
                tickfont: { size: 10 },
                showgrid: true,
                zeroline: false,
                automargin: true,
                gridcolor: cssVar("--nav-active"),
            },
        plot_bgcolor: cssVar("--snd-bg-color"),
        paper_bgcolor: "rgba(0,0,0,0)",
        font: {
            color: cssVar("--text-color"),
            family: "Inter, system-ui, Helvetica, sans-serif",
        },
        autosize: true,
    };
}
