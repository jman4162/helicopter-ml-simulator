import { describe, it, expect } from 'vitest';
import { TOUR } from './tour';
import { AIRSHOW_SEQUENCE, MANEUVER_IDS } from '../control/maneuvers';

const MODES = new Set(['manual', 'hover', 'gust', 'flip', 'learning', 'sysid', 'airshow']);

describe('guided tour data', () => {
  it('every step is well-formed with a known mode', () => {
    expect(TOUR.length).toBeGreaterThan(4);
    for (const s of TOUR) {
      expect(s.title.length).toBeGreaterThan(0);
      expect(s.body.length).toBeGreaterThan(0);
      expect(MODES.has(s.mode)).toBe(true);
    }
  });

  it('has unique step tags and covers all three pillars', () => {
    const tags = TOUR.map((s) => s.tag);
    expect(new Set(tags).size).toBe(tags.length);
    const modes = new Set(TOUR.map((s) => s.mode));
    expect(modes.has('manual')).toBe(true); // physics
    expect(modes.has('hover')).toBe(true); // control
    expect(modes.has('learning')).toBe(true); // learning
    expect(modes.has('sysid')).toBe(true); // system ID
  });
});

describe('airshow sequence', () => {
  it('chains only valid maneuvers', () => {
    expect(AIRSHOW_SEQUENCE.length).toBeGreaterThan(1);
    for (const m of AIRSHOW_SEQUENCE) expect(MANEUVER_IDS).toContain(m);
  });
});
