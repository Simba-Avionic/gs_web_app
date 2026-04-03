<script context="module">
  import { enumMappings } from "./stores/enums";

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

  export function getStateString(statusCode, enumName = "SIMBA_ROCKET_STATE") {
    let mappings = {};
    const unsubscribe = enumMappings.subscribe((e) => (mappings = e));
    unsubscribe();

    if (!mappings[enumName]) {
      console.warn(`Enum ${enumName} not found`);
      return String(statusCode);
    }

    const enumDef = mappings[enumName];
    const name = enumDef.values[statusCode];

    if (!name) return "UNKNOWN_STATE";

    return name.replace(new RegExp(`^${enumName}_`), "");
  }

  export function stripSimbaPrefix(str) {
    if (typeof str !== "string") {
      return String(str);
    }

    return str.replace(/simba_/g, "");
  }
</script>
