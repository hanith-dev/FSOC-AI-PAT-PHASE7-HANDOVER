/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        // Sleek dark-mode color theme
        dark: {
          bg: "#0B0F17",
          card: "#151D2A",
          border: "#243145",
          primary: "#3B82F6",
          accent: "#10B981"
        }
      }
    },
  },
  plugins: [],
}
