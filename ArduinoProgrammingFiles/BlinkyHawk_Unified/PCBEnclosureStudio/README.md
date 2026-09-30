# PCB Enclosure Studio

A local, parametric enclosure CAD app for KiCad boards and detailed STEP PCB assemblies, with face attachments and printable solid exports. Version 0.2.

## Open the app

Double-click **Start Enclosure Studio.cmd** in this folder. It opens **http://127.0.0.1:8766** in your browser. The dependencies are already installed on this computer in `.runtime`.

**Stop Enclosure Studio.cmd** stops this app's background service. Closing the browser leaves the service running so you can reopen it quickly. Nothing is added to Windows startup.

Files and geometry stay on your computer. The editor binds to the local loopback address, uses local scripts and fonts, and has no account, cloud processing, analytics, or runtime CDN dependency. An internet connection is required only to install dependencies on a new computer.

## First enclosure

1. Click **Import PCB** and select a `.kicad_pcb`, `.step`, or `.stp` file. The welcome screen includes both KiCad and detailed STEP examples of BlinkyHawk V3b. A shell, floor, lid, and locating lip are generated automatically.
2. Select **Enclosure** in the tree. Set clearance, wall/floor thickness, space above and below the PCB, and lid dimensions. Choose a rounded rectangle or an offset of the PCB outline.
3. For a STEP import, select the PCB in the tree and verify the detected substrate body and thickness. STEP retains the actual component solids, including modeled connectors. For a KiCad import, review component envelopes. Select a component in the tree or viewport, then adjust its height and envelope dimensions. These are mechanical estimates from courtyard/fabrication drawings, not imported component STEP bodies.
4. Select a connector, then click **Window**. Its opening projects to the nearest wall. Adjust the selected component feature, target wall, orientation, width, height, depth, and offsets. The feature follows the component when its placement changes or a revised board with the same footprint UUID is imported.
5. Add **Hole**, **Standoff**, **Boss**, **Vents**, **Lettering**, or **Sketch** features. Each has an add/remove operation, target part, attachment, local translation, and rotation. Sketches are dimensioned polygon coordinates, one U/V pair per line.
6. Use **Standoffs from PCB holes** for footprints identified as mounting holes or suitable non-plated holes. Verify the screw bore and post diameter for your hardware. Wire and connector holes are not automatically treated as mounting holes.
7. Inspect **Fit check**, then **Export parts**. Choose 3MF, STL, or STEP, and either a single part or both in a ZIP. Failed features block export. Overlap warnings remain available for engineering judgment.
8. **Save project** downloads a portable `.pcbshell` project containing the PCB data, complete feature history, the original STEP PCB assembly when used, and imported reference assets. **Open** restores it. Browser autosave is convenient, but a portable project is the durable copy.

**New** starts a blank workspace; Undo restores the previous project during that session. Importing another PCB into an existing project replaces its board and retains its features, planes, and references. If a referenced component no longer exists, select a replacement attachment in the failed feature's properties.

## Attachments and planes

- **World origin:** select XY, XZ, or YZ (or reversed normal), then enter U/V/N offsets and rotations. Millimeters throughout.
- **PCB component:** footprint origin, body center, or any envelope face. Orientation follows the footprint unless a world plane is selected.
- **Component pad:** pad position and footprint orientation. **PCB hole:** drill position at the inside floor, useful for posts.
- **Enclosure surface:** floor, rim, outside of lid, or rectangular bounding walls.
- **Construction plane:** an independent plane that can follow another plane, a component, a surface, or a reference face. Circular attachments are reported as errors.
- **Picked STEP face:** import STEP, position it, select **Attach to face**, and click a flat face. With a feature/plane selected, this changes its attachment; otherwise it creates a new construction plane. Reference transforms carry attached features with them.

The origin is the lower-left corner of the imported board's XY bounds; the PCB bottom is Z = 0. KiCad screen Y is reversed to give right-handed CAD coordinates. U and V lie in the attachment plane; N follows its normal. Red, green, and blue axes show U, V, and N on the selected feature/plane. `Project origin to` moves the attachment origin onto an enclosure surface while retaining the chosen orientation.

For an exact connector opening, import its STEP model, align it to the PCB, pick its mating face, and use its plane for a window or hole. Select an enclosure surface under `Project origin to` if the connector face does not reach the wall, and give the cutting feature enough depth to pass through it.

## Detailed PCB assemblies from STEP

**Import PCB** accepts STEP/STP directly, without a companion KiCad file. The importer retains the solid bodies, assembly names, and available body colors, detects the broadest flat body as the substrate, normalizes its bottom to Z = 0, and creates the enclosure from its outline. Inspect the substrate selection in PCB properties; multi-board assemblies can require choosing the main board manually. **Flip board side** reverses the mounting side if the inferred normal is wrong.

Your `BlinkyHawkModel_v3b.step` contains 301 solids grouped into 41 components, including **USB TYPE C PORT**. Its main PCB measures 25.4 × 53.34 mm and is **1.51 mm thick in the supplied STEP**. The prepared `examples/BlinkyHawk_V3b_STEP.pcbshell` embeds that source and opens with the detailed assembly.

Select the USB component to create a projected window from its bounds, or use **Pick an exact face** / **Attach to face** to create an attachment to a specific planar surface. Directional component attachments use measured bounding faces; explicitly picked faces use the actual B-rep surface. Curved surfaces are not planar anchors. STEP component dimensions are read-only; reposition or revise components in the source CAD, then reimport. Source replacement invalidates old STEP face anchors so they can be re-picked rather than silently moving to a different surface.

Detailed PCB solids participate in fit checks and enclosure height calculation. Substrate holes are retained; circular openings at least 1.8 mm across are offered as possible mounting holes. STEP lacks KiCad's pad numbers and plating metadata, so inspect these before generating posts. Imported reference files under **Add reference** remain visual aids outside fit checks.

## Reference imports

**STEP/STP** imports solid bodies and exposes planar face attachments. **STL/3MF** imports display meshes, positioned by translation and rotation; create a construction plane for their features. Use **Add reference** for separate auxiliary models. Use **Import PCB** to make a STEP assembly the actual PCB source, with its substrate outline and detailed components. Auxiliary references require an imported KiCad or STEP board.

Models arrive in their original coordinates. **Center on PCB / bottom at Z = 0** supplies a starting alignment, resets rotation, and does not infer the physical component position. STL is unitless and interpreted as millimeters; 3MF declared units are converted to millimeters. STEP uses the CAD importer's unit conversion.

`examples/Demo_connector_reference.step` is a simple demonstration body for testing face selection, **not an accurate model of the BlinkyHawk USB connector**. The included BlinkyHawk example generates a new general-purpose enclosure; it does not recreate the previously completed, battery-specific V3b print files.

## Print and fit behavior

Each exported shell/lid must be a connected valid solid. STL and 3MF exports are tessellated at 0.035 mm linear tolerance and checked for watertightness. Their lowest vertex sits at Z = 0, and the lid is flipped so its outside faces down. Review orientation and supports for raised lettering, protrusions, and bridging. STEP retains the assembly coordinates and analytic solids.

Fit checking intersects the shell and seated lid with the actual substrate and component solids for STEP boards, or with the substrate and editable component envelopes for KiCad boards. Imported reference models, solder, wires, cable overmolds, batteries, and external switches are **not** automatically included. Add dimensioned keepout/reference geometry for visual inspection and make a fit print. Shell/lid interference is an export-blocking error; estimated component overlaps are warnings.

## Current boundaries

- A focused feature modeler, not a full SolidWorks/Inventor replacement: no general sketch-constraint solver, assemblies/mate solver, face-driven fillet tool, snap latch generator, or native SLDPRT/IPT import.
- KiCad 6+ board outlines made from lines, arcs, circles, polygons, and rectangles. Outline curves are sampled to roughly 0.015 mm chord error. Bezier outlines, open contours, and multiple disjoint boards are rejected with a message.
- KiCad component heights are estimates; external KiCad 3D libraries are not resolved. Import the populated STEP assembly to use its detailed geometry. STEP accuracy is limited to the supplied model; it cannot supply components absent from that file.
- Source footprints are matched by UUID on board replacement. STEP attachments belong to the imported immutable reference; replacing it with a different file requires re-picking faces.
- Outline-following shells use the rectangular bounds for wall projections. Position cuts manually for slanted/curved walls.
- One local workspace per browser profile with 60 undo steps. Limits: 150 features, 40 construction planes, 64 MB per uploaded file, and 400,000 triangles per mesh reference body. Large assemblies may rebuild slowly.

## Development and validation

Python 3.12/3.13, 64-bit, recommended. On a fresh computer, run **Install dependencies.cmd**, then the launcher. The app prefers the bundled Codex Python if present and falls back to `python` on PATH. CAD dependencies live in this folder; installing them does not change the system Python environment.

Run tests from this folder with `python run_tests.py`. Use the bundled Python executable if `python` is not on PATH. Set `PCB_STUDIO_PORT` before launching to use another local port.

The automated suite covers the actual V3b board, analytic solids, connected/watertight exports in three formats, rotated component and pad attachments, planar STEP transforms, custom plane chains and cycle errors, slot orientation, feature suppression, portable projects, collisions, and cross-origin request rejection. Browser verification covers model loading, component search/selection, feature creation, dimension changes, undo, STEP import, planar face picking, and portable download/open.

The backend is FastAPI with CadQuery/Open CASCADE; the viewport is Three.js. Vendored Three.js and Lucide licenses are in `static/vendor`. Python package licenses accompany the app-local installed distributions. Mechanical parsing follows the [KiCad board format](https://dev-docs.kicad.org/en/file-formats/sexpr-pcb/); solid export uses [CadQuery](https://cadquery.readthedocs.io/en/latest/importexport.html).
