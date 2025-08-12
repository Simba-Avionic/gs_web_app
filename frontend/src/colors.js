// Based on colors inside app.css

export function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export const colors = [
  "#388729", "#5aa54b", "#99d88d", "#caf2c2", "#77be69",  // green
  "#e0b400", "#f3cc0f", "#ffed52", "#fff899", "#fade2b",  // yellow
  "#f24865", "#c41934", "#de304d", "#ff7289", "#ffa6b4",  // red
  "#5694f2", "#1857b8", "#3373d9", "#8ab7ff", "#c0d8ff",  // blue
  "#ff9830", "#fa6400", "#ff7809", "#feb356", "#fecb7c",  // orange
  "#b876d9", "#8936b2", "#a352cb", "#cb95e5", "#deb6f2"   // purple
];
