/** Shared review quality classification and palette. */

export const RMS_COLORS = Object.freeze({
  low: 0x4bcf7d,
  medium: 0xf0a53a,
  high: 0xef5f80,
  neutral: 0x8c98a8,
});

/** Return the same RMS band used by both the sidebar and the 3D scene. */
export function rmsBand(value) {
  if (value == null || typeof value === "boolean") return null;
  if (typeof value === "string" && value.trim() === "") return null;
  if (typeof value !== "number" && typeof value !== "string") return null;
  const number = Number(value);
  if (!Number.isFinite(number)) return null;
  return number < 5 ? "low" : number < 10 ? "medium" : "high";
}

export function rmsColorHex(value, colored = true) {
  if (!colored) return RMS_COLORS.neutral;
  const band = rmsBand(value);
  return band == null ? RMS_COLORS.neutral : RMS_COLORS[band];
}
