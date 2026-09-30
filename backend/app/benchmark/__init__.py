"""
Scientific Benchmark Engine for FSOC-AI-PAT.
Headless, deterministic, fixed-timestep evaluation framework.
"""
from app.benchmark.runner import BenchmarkEngine, BenchmarkRunResult
from app.benchmark.scenarios import BenchmarkScenario, SCENARIO_REGISTRY

__all__ = ["BenchmarkEngine", "BenchmarkRunResult", "BenchmarkScenario", "SCENARIO_REGISTRY"]
