import os
import pytest
from fastapi.testclient import TestClient
from app.main import app, RESULTS_DIR

@pytest.fixture
def client():
    return TestClient(app)

def test_api_scenarios(client):
    response = client.get("/api/scenarios")
    assert response.status_code == 200
    data = response.json()
    assert isinstance(data, list)
    assert len(data) >= 10  # Predefined scenarios count

def test_api_experiments_results(client):
    response = client.get("/api/experiments/results")
    assert response.status_code == 200
    assert isinstance(response.json(), list)

def test_api_experiments_compare_missing_ids(client):
    response = client.get("/api/experiments/compare?ids=")
    assert response.status_code == 200
    assert response.json() == []

def test_api_experiments_download_endpoints_404(client):
    # Testing fake id returns error response
    response = client.get("/api/experiments/results/nonexistent_id_abc/pdf")
    assert response.status_code == 200 or response.status_code == 404
    if response.status_code == 200:
        assert response.json()["status"] == "error"
        
    response = client.get("/api/experiments/results/nonexistent_id_abc/csv")
    assert response.status_code == 200 or response.status_code == 404
    if response.status_code == 200:
        assert response.json()["status"] == "error"
