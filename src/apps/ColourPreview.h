// SPDX-License-Identifier: MIT
#ifndef COLOURPREVIEW_H
#define COLOURPREVIEW_H
// ---------------------------------------------------------------------------
// The colours page's preview (Ui::menuPreview): what the LED cursor's colour
// mapping looks like as the levers stand - the wheel as mapped (hue start,
// turns), the cursor's colour across the scheme's data (the height with its
// white plateau, the sureness, the lean's compass, the rows), and the other
// colours in play: the tail's, the poles', the dimming. Drawn under the
// page's items and over a tweak's strip, so a value is edited with its
// effect in view. Kevin, 2026-09-26: "a visual editor with previews of the
// coloring and color spectrums".
// ---------------------------------------------------------------------------
class GFXcanvas16;

int colourPreviewRows( const char* page ); // 3 for the colours page, 0 for the others
void colourPreviewDraw( GFXcanvas16* canvas, const char* page, int x, int y, int w, int h );

#endif // COLOURPREVIEW_H
