/**
 * PID gains as ROS 2 parameters of /robot_bridge (gains.<loop>.<term>), read and written through
 * the node's standard rcl_interfaces services.
 */

export const LOOPS = ["pitch", "velocity", "yaw_rate"] as const;
export const TERMS = ["kp", "ki", "kd"] as const;
export type Loop = (typeof LOOPS)[number];
export type Term = (typeof TERMS)[number];
export type Gains = Record<Loop, Record<Term, number>>;

export const LOOP_LABELS: Record<Loop, string> = {
  pitch: "Pitch",
  velocity: "Velocity",
  yaw_rate: "Yaw rate",
};

/** rcl_interfaces/msg/ParameterType.PARAMETER_DOUBLE */
export const PARAMETER_DOUBLE = 3;

export function parameterName(loop: Loop, term: Term): string {
  return `gains.${loop}.${term}`;
}

export const GAIN_PARAMETER_NAMES: string[] = LOOPS.flatMap((loop) =>
  TERMS.map((term) => parameterName(loop, term)),
);

interface ParameterValue {
  type: number;
  double_value?: number;
}

/** Builds Gains from a GetParameters response for GAIN_PARAMETER_NAMES (in that order). */
export function gainsFromResponse(values: ParameterValue[]): Gains {
  if (values.length !== GAIN_PARAMETER_NAMES.length) {
    throw new Error(`expected ${GAIN_PARAMETER_NAMES.length} values, got ${values.length}`);
  }
  const gains = {} as Gains;
  let index = 0;
  for (const loop of LOOPS) {
    gains[loop] = { kp: 0, ki: 0, kd: 0 };
    for (const term of TERMS) {
      const value = values[index++];
      if (value.type !== PARAMETER_DOUBLE || typeof value.double_value !== "number") {
        throw new Error(`${parameterName(loop, term)} is not a double`);
      }
      gains[loop][term] = value.double_value;
    }
  }
  return gains;
}

/** SetParameters request body for one gain. */
export function setGainRequest(loop: Loop, term: Term, value: number) {
  return {
    parameters: [
      { name: parameterName(loop, term), value: { type: PARAMETER_DOUBLE, double_value: value } },
    ],
  };
}

/** Validates user input the same way the bridge does: finite and non-negative. */
export function parseGain(text: string): number | null {
  if (text.trim() === "") {
    return null;
  }
  const value = Number(text);
  return Number.isFinite(value) && value >= 0 ? value : null;
}
