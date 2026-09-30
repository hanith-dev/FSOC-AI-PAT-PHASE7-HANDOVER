# AGENTS.md - MANDATORY WORKSPACE RULES

## UI / UX ENGINEERING CONSTRAINTS (STRICTLY HARDCODED)
The following 30 anti-patterns are strictly forbidden across all frontend files, interfaces, and designs:

1. NO harsh gradients (use flat, solid, high-contrast matte surfaces)
2. NO lucide icons (use clean technical indicators, minimal SVGs, or raw unicode/ascii markers)
3. PURE WHITE BACKGROUND EXPLICIT OVERRIDE: Per explicit user directive on 2026-09-30, complete background is configured to pure white (#ffffff) with high-contrast slate-900 typography and crisp 1px borders. (Original constraint: NO pure white background).
4. NO rainbow coloring (restrain color strictly to functional telemetry status: green=lock, amber=coasting/degraded, red=alarm)
5. NO drop shadows (no box-shadow, shadow-lg, shadow-2xl; use crisp 1px borders)
6. NO 3 feature cards in a row
7. NO emojis
8. NO liquid glass / glassmorphism (no backdrop-blur or frosted glass effects)
9. NO em dashes (—) (use standard hyphens `-` or colons `:`)
10. NO inter/geist/space grotesk fonts (use standard monospace `font-mono` or system sans)
11. NO colored left stripes
12. NO fake testimonials
13. NO bento grids
14. NO terminal window frames (no fake window dot headers)
15. NO "it's not x, it's y" copywriting
16. NO checkmark bullets
17. NO 3 pricing tiers
18. NO absence of real product demos (interfaces must feature live, functional instrument controls and real-time data)
19. NO soft corner radius (use crisp technical borders: `rounded-none` or `rounded-sm`)
20. NO purple and black color palettes
21. NO missing skeleton loaders (always handle async/loading states cleanly)
22. NO radial orbs / background glowing circles
23. NO dot grids
24. NO sparkle icons
25. NO animated arrows
26. NO missing TOS / Operational compliance notice
27. NO missing privacy policy notice
28. NO hover animations / bounce / active scale effects
29. NO neon colors or neon glow filters
30. NO basic pastel colors
