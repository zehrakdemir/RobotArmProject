"""3D gorunum icin robot modeli. Robot, ROS2 ve SDK GEREKTIRMEZ."""

from fairino_cobot.robot_modeli import model_paketi, paket_ici_dosya, paket_uri_coz, urdf_ozeti

URDF = """<?xml version="1.0"?>
<robot name="deneme_robot">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://desc/meshes/base_link.STL"/></geometry>
      <material name=""><color rgba="0.9 0.9 0.9 1"/></material>
    </visual>
  </link>
  <link name="shoulder_link">
    <visual>
      <geometry><mesh filename="package://desc/../../etc/passwd.STL"/></geometry>
    </visual>
  </link>
  <link name="upperarm_link">
    <visual>
      <geometry><mesh filename="file:///etc/shadow.STL"/></geometry>
    </visual>
  </link>
  <link name="bos_link"/>
  <joint name="j1" type="revolute">
    <origin xyz="0 0 0.14" rpy="1.5708 0 0"/>
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
"""


def test_urdf_kinematik_zinciri_okunuyor():
    ozet = urdf_ozeti(URDF)

    assert ozet["name"] == "deneme_robot"
    assert [link["name"] for link in ozet["links"]] == [
        "base_link", "shoulder_link", "upperarm_link", "bos_link"]
    j1 = ozet["joints"][0]
    assert j1["parent"] == "base_link" and j1["child"] == "shoulder_link"
    assert j1["xyz"] == [0.0, 0.0, 0.14] and j1["rpy"] == [1.5708, 0.0, 0.0]
    assert j1["axis"] == [0.0, 0.0, 1.0]
    assert ozet["links"][0]["rgba"] == [0.9, 0.9, 0.9, 1.0]


def test_yalnizca_package_uri_cozuluyor():
    assert paket_uri_coz("package://desc/meshes/a.STL") == ("desc", "meshes/a.STL")
    assert paket_uri_coz("file:///etc/shadow") is None
    assert paket_uri_coz("/etc/shadow") is None
    assert paket_uri_coz("package://desc") is None
    assert paket_uri_coz(None) is None


def test_paket_klasorunden_disari_cikilamiyor(tmp_path):
    assert paket_ici_dosya(tmp_path, "meshes/a.STL") == (tmp_path / "meshes/a.STL").resolve()
    assert paket_ici_dosya(tmp_path, "../disari.STL") is None
    assert paket_ici_dosya(tmp_path, "meshes/../../disari.STL") is None


def test_model_paketi_yalnizca_paket_icindeki_stl_dosyalarini_sunuyor(tmp_path):
    paket = tmp_path / "share" / "desc"
    (paket / "meshes").mkdir(parents=True)
    (paket / "meshes" / "base_link.STL").write_bytes(b"solid x")
    # Paketin DISINDA ama '..' ile ulasilabilecek bir dosya:
    (tmp_path / "etc").mkdir()
    (tmp_path / "etc" / "passwd.STL").write_bytes(b"gizli")

    ozet, dosyalar = model_paketi(URDF, lambda ad: str(paket) if ad == "desc" else None)

    meshler = {link["name"]: link["mesh"] for link in ozet["links"]}
    assert meshler == {"base_link": "/api/model/mesh/0", "shoulder_link": None,
                       "upperarm_link": None, "bos_link": None}
    assert dosyalar == [(paket / "meshes" / "base_link.STL").resolve()]
    assert all("mesh_uri" not in link for link in ozet["links"]), "ham yol arayuze gitmemeli"


def test_paket_bulunamazsa_mesh_yok():
    ozet, dosyalar = model_paketi(URDF, lambda ad: None)

    assert dosyalar == []
    assert all(link["mesh"] is None for link in ozet["links"])
