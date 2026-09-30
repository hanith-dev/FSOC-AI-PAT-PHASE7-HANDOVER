# Strict UI/UX Engineering Rules

These rules are permanent constraints for all frontend and interface designs across this workspace.
Never violate or re-introduce any of these 30 anti-patterns:

1. **NO harsh gradients** - Use solid, flat, high-contrast matte surfaces.
2. **NO lucide icons** - Use clean, minimal technical SVGs or concise textual/ASCII status indicators (e.g., `[●]`, `[+]`, `[-]`, `[RST]`, `[RUN]`).
3. **Pure White Background (User Overridden)** - Per explicit user directive on 2026-09-30, the entire interface background is set to pure white (`#ffffff`) with high-contrast slate-900 typography and crisp 1px borders. (Default constraint: NO pure white background).
4. **NO rainbow coloring** - Strictly restrain color to functional telemetry indicators (e.g., emerald for nominal/lock, amber for degraded/coasting, rose for lost/alarm).
5. **NO drop shadows** - Zero `box-shadow`, `shadow-lg`, `shadow-2xl`, or blur shadows. Use crisp, high-precision 1px borders (`border border-slate-800`).
6. **NO 3 feature cards in a row** - Design authentic operational instruments, telemetry readouts, and engineering consoles instead of generic marketing card triplets.
7. **NO emojis** - Strictly forbidden in professional engineering and aerospace interfaces.
8. **NO liquid glass / glassmorphism** - No `backdrop-blur`, frosted glass, or translucent milky panels. Use solid, opaque matte surfaces.
9. **NO em dashes (—)** - Use standard hyphens (`-`), colons (`:`), or pipe separators (`|`).
10. **NO inter/geist/space grotesk fonts** - Use system monospace (`ui-monospace`, `monospace`, `font-mono`) or clean technical sans (`system-ui`, `sans-serif`).
11. **NO colored left stripes** - Avoid arbitrary accent stripes on borders or card margins.
12. **NO fake testimonials** - Never include mock reviews, quotes, or marketing testimonials.
13. **NO bento grids** - Avoid trendy asymmetric marketing tiles. Use structured engineering panels aligned to operational hierarchy.
14. **NO terminal window aesthetic** - Avoid mock macOS window dots (red/yellow/green circles) or fake hacker consoles.
15. **NO "it's not x, it's y" copywriting** - Use direct, precise engineering terminology.
16. **NO checkmark bullets** - Use standard tabular readouts or clean bullet/hyphen notations.
17. **NO 3 pricing tiers** - Strictly prohibited in functional systems.
18. **NO absence of real product demos** - Always provide real, functional controls that operate the underlying physical/algorithmic engine with real-time feedback.
19. **NO soft corner radius** - Avoid `rounded-xl`, `rounded-2xl`, `rounded-3xl`. Use crisp technical borders: `rounded-none` or `rounded-sm`.
20. **NO purple and black color schemes** - Avoid violet/purple thematic combinations. Use technical slate, graphite, and aerospace dark navy tones.
21. **NO missing skeleton loaders** - Ensure all asynchronous data views handle loading states gracefully without raw flashes or layout shifts.
22. **NO radial orbs** - No background glowing orbs, blur blobs, or decorative light circles.
23. **NO dot grids** - No decorative background polka-dot grids or canvas particle meshes.
24. **NO sparkle icons** - No AI magic sparkles, stars, or decorative glitter.
25. **NO animated arrows** - No bouncing or sliding arrows.
26. **NO missing TOS / Legal links** - Include standard system operational terms and specifications when applicable.
27. **NO missing privacy policy** - Include standard privacy and data transmission notice when applicable.
28. **NO hover animations / transforms** - No `active:scale-95`, bouncy hover zooms, or floating animations.
29. **NO neon colors or neon glows** - Avoid radioactive greens, glowing cyan drop shadows, or glow filters. Use clean, calibrated functional colors.
30. **NO basic pastel colors** - Avoid washed-out baby blues, pale pinks, or chalky tones.
