import http.client

import pytest

pytest.importorskip("PyQt6.QtWidgets")

from beambot_gui.visualization_panel import _MeshServer


def _get(port, path):
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
    connection.request("GET", path)
    response = connection.getresponse()
    body = response.read()
    headers = dict(response.getheaders())
    connection.close()
    return response.status, body, headers


@pytest.mark.loopback
def test_mesh_server_contains_every_mounted_root(tmp_path):
    resources = tmp_path / "resources"
    urdf = tmp_path / "urdf"
    package = tmp_path / "package"
    for directory in (resources, urdf, package):
        directory.mkdir()
    (resources / "viewer.html").write_text("viewer")
    (urdf / "robot.urdf").write_text("robot")
    (package / "mesh.stl").write_text("mesh")
    secret = tmp_path / "secret.txt"
    secret.write_text("secret")
    for root in (resources, urdf, package):
        (root / "escape").symlink_to(secret)

    server = _MeshServer({"pkg": package}, resources, urdf)
    server.start()
    try:
        assert _get(server.port, "/__static__/viewer.html?version=1#viewer")[:2] == (
            200,
            b"viewer",
        )
        assert _get(server.port, "/__urdf__/robot.urdf")[:2] == (200, b"robot")
        assert _get(server.port, "/pkg/mesh.stl")[:2] == (200, b"mesh")
        assert _get(server.port, "/viewer.html")[:2] == (200, b"viewer")

        escapes = (
            "/__static__/%2e%2e%2fsecret.txt?version=1",
            "/__urdf__/../secret.txt",
            "/pkg/%2e%2e/secret.txt?version=1",
            "/%2e%2e/secret.txt",
            f"/__static__//{str(secret).lstrip('/')}",
            "/__static__/escape",
            "/__urdf__/escape",
            "/pkg/escape",
            "/escape",
        )
        for path in escapes:
            status, body, headers = _get(server.port, path)
            assert status == 404 and b"secret" not in body
            assert "Access-Control-Allow-Origin" not in headers
    finally:
        server.stop()

    assert server.server.fileno() == -1
    assert not server._thread.is_alive()
