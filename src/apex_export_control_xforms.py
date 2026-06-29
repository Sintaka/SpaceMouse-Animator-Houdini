"""
APEX Script Node -- export all control world transforms as geometry
===================================================================
Drop this into an APEX Python Script node, wire its single output to
whatever needs control world transforms.

Output: geometry points, one per TransformObject node.
  - @name (string) : control name
  - @xform (4x4 matrix attr) : world transform (Matrix4 → 16 floats)
  - @localxform (4x4 matrix attr) : local transform
  - @restlocal (4x4 matrix attr) : rest pose local transform
  - P : world-space translation extracted from xform
"""
import _apex


def cook(graph, inputs, outputs, parms, report):
    """Standard APEX Python Script callback"""
    geo = _apex.Geometry()

    # Tag to filter: controls are TransformObject nodes
    # Common tags: 'control', 'Control', 'fk', 'ik', 'TransformObject'
    # You can tweak the tag filter as needed
    tag_patterns = ('%type(TransformObject)',)

    for pattern in tag_patterns:
        try:
            nodes = graph.findNode(pattern)
        except Exception:
            nodes = []

        for node in nodes:
            try:
                # Read world xform from the TransformObject's output port
                xf = node.xform_out  # Matrix4 — world transform
                if xf is None:
                    continue

                name = node.name()

                # Also try to read local and rest local transforms
                local_xf = None
                rest_local = None
                try:
                    local_xf = node.localxform_out
                except:
                    pass
                try:
                    rest_local = node.restlocal_out
                except:
                    pass

                pt = geo.addPoint()
                pt.setAttrib('name', name)
                pt.setAttrib('xform', xf)
                if local_xf: pt.setAttrib('localxform', local_xf)
                if rest_local: pt.setAttrib('restlocal', rest_local)

                # Extract world position as P
                pt.setPosition(_apex.Vector3(xf[12], xf[13], xf[14]))

            except Exception:
                pass

    outputs[0] = geo
    return True
