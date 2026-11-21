# YOLO26n model notice

`SoundGuide/YOLO26n.mlpackage` is a Core ML export of Ultralytics YOLO26n object-detection weights. Ultralytics provides the model under the [GNU Affero General Public License v3.0](AGPL-3.0.txt). The model was converted to a 640 × 480 portrait Core ML pipeline with FP16 weights and non-maximum suppression for Apple Vision.

- [Upstream source and license (Ultralytics v8.4.216)](https://github.com/ultralytics/ultralytics/tree/v8.4.216)
- [Original YOLO26n checkpoint](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt)
- [Export script and checkpoint checksum](../scripts/export-yolo26.sh)

This notice covers the bundled model. Before distributing a build with the model, make sure the combined app's licensing and source availability satisfy the AGPL, or obtain a suitable commercial license.
