
# lanelet2anchors

This package enables the generation of diverse map-based anchor paths for a given lanelet or vehicle pose. It was developed as part of our paper <a href='https://openaccess.thecvf.com/content/CVPR2023W/E2EAD/html/Naumann_Lanelet2_for_nuScenes_Enabling_Spatial_Semantic_Relationships_and_Diverse_Map-Based_CVPRW_2023_paper.html'>Lanelet2 for nuScenes: Enabling Spatial Semantic Relationships and Diverse Map-based Anchor Paths</a>. For details check our <a href='https://felixhertlein.github.io/lanelet4nuscenes'>project page</a>.

## Installation

Use Python >= 3.8 and `pip` to install

```shell
pip install lanelet2anchors
```

## Usage

Check the [notebooks](notebooks/example.ipynb) in the [documentation](/) for details and examples.

### Generate anchors for given vehicle
```python
from lanelet2anchors import AnchorGenerator

lanelet_map = AnchorGenerator(
    osm_file=osm_file, origin_latitude=49.00345654351, origin_longitude=8.42427590707
)

anchors = lanelet_map.create_anchors_for_vehicle(
    vehicle_pose=X,
    anchor_length=100,
    num_anchors: 5,
)
```

### Generate anchors for a given Lanelet
```python
from lanelet2anchors import AnchorGenerator

lanelet_map = AnchorGenerator(
    osm_file=osm_file, origin_latitude=49.00345654351, origin_longitude=8.42427590707
)

anchors = lanelet_map.create_anchors_for_lanelet(
    lanelet_id=lanelet_map.lanelet_ids[0], anchor_length=100
)
```

## Citation

If you use this code for scientific research, please consider citing


```bibtex
@InProceedings{naumannHertleinLanelet2NuScenes,
    author    = {Naumann, Alexander and Hertlein, Felix and Grimm, Daniel and Zipfl, Maximilian and Thoma, Steffen and Rettinger, Achim and Halilaj, Lavdim and Luettin, Juergen and Schmid, Stefan and Caesar, Holger},
    title     = {Lanelet2 for nuScenes: Enabling Spatial Semantic Relationships and Diverse Map-Based Anchor Paths},
    booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR) Workshops},
    month     = {June},
    year      = {2023},
    pages     = {3247-3256}
}
```


## License

This project is licensed under [CC-BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode). Development by [FZI Forschungszentrum Informatik](https://www.fzi.de/) and all rights reserved by [Robert Bosch GmbH](https://www.bosch.com/).