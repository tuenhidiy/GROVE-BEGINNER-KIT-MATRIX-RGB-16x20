#ifndef FONT8X16_H
#define FONT8X16_H

static const uint8_t font8x16[186][16] PROGMEM = {
/*   (32) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* ! (33) */ {0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00},
/* " (34) */ {0x00, 0x00, 0x00, 0xD8, 0xD8, 0xD8, 0xD8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* # (35) */ {0x00, 0x00, 0x00, 0x00, 0x12, 0x12, 0x7F, 0x7F, 0x24, 0xFE, 0xFE, 0x48, 0x48, 0x00, 0x00, 0x00},
/* $ (36) */ {0x00, 0x00, 0x10, 0x10, 0x7C, 0xD2, 0xD0, 0xD0, 0x7C, 0x16, 0x16, 0x96, 0x7C, 0x10, 0x10, 0x00},
/* % (37) */ {0x00, 0x00, 0x00, 0x00, 0x78, 0xCC, 0xCC, 0xCD, 0x7A, 0x05, 0x09, 0x11, 0x00, 0x00, 0x00, 0x00},
/* & (38) */ {0x00, 0x00, 0x00, 0x00, 0x3C, 0x66, 0x66, 0x66, 0x3C, 0x66, 0x63, 0x63, 0x3D, 0x00, 0x00, 0x00},
/* ' (39) */ {0x00, 0x00, 0x00, 0x60, 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* ( (40) */ {0x00, 0x00, 0x00, 0x18, 0x30, 0x30, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x30, 0x30, 0x18},
/* ) (41) */ {0x00, 0x00, 0x00, 0x60, 0x30, 0x30, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x30, 0x30, 0x60},
/* * (42) */ {0x00, 0x00, 0x00, 0x18, 0x5A, 0x3C, 0x5A, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* + (43) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x3F, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00},
/* , (44) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0xC0, 0xC0, 0x00},
/* - (45) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* . (46) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00},
/* / (47) */ {0x00, 0x00, 0x00, 0x04, 0x04, 0x08, 0x08, 0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x80, 0x80, 0x00},
/* 0 (48) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* 1 (49) */ {0x00, 0x00, 0x00, 0x00, 0x18, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00, 0x00, 0x00},
/* 2 (50) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0x86, 0x86, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xFE, 0x00, 0x00, 0x00},
/* 3 (51) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0x86, 0x06, 0x06, 0x3C, 0x06, 0x06, 0x86, 0x7C, 0x00, 0x00, 0x00},
/* 4 (52) */ {0x00, 0x00, 0x00, 0x00, 0x0C, 0x1C, 0x2C, 0x4C, 0x8C, 0xFE, 0x0C, 0x0C, 0x0C, 0x00, 0x00, 0x00},
/* 5 (53) */ {0x00, 0x00, 0x00, 0x00, 0x7E, 0x60, 0x60, 0x7C, 0x06, 0x06, 0x06, 0x86, 0x7C, 0x00, 0x00, 0x00},
/* 6 (54) */ {0x00, 0x00, 0x00, 0x00, 0x3C, 0x60, 0xC0, 0xFC, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* 7 (55) */ {0x00, 0x00, 0x00, 0x00, 0xFE, 0x06, 0x0C, 0x0C, 0x18, 0x18, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00},
/* 8 (56) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7C, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* 9 (57) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x0C, 0x78, 0x00, 0x00, 0x00},
/* : (58) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00},
/* ; (59) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x60, 0x60, 0xC0, 0xC0, 0x00},
/* < (60) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x06, 0x18, 0x60, 0x60, 0x18, 0x06, 0x01, 0x00, 0x00, 0x00},
/* = (61) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* > (62) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x18, 0x06, 0x01, 0x01, 0x06, 0x18, 0x60, 0x00, 0x00, 0x00},
/* ? (63) */ {0x00, 0x00, 0x00, 0x00, 0x78, 0x8C, 0x0C, 0x18, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00},
/* @ (64) */ {0x00, 0x00, 0x00, 0x00, 0x3E, 0x41, 0x9E, 0xB6, 0xB6, 0xB6, 0xB6, 0x9B, 0x40, 0x3F, 0x00, 0x00},
/* A (65) */ {0x00, 0x00, 0x00, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* B (66) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0xC6, 0xC6, 0xC6, 0xFC, 0xC6, 0xC6, 0xC6, 0xFC, 0x00, 0x00, 0x00},
/* C (67) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0xC2, 0xC2, 0xC0, 0xC0, 0xC0, 0xC2, 0xC2, 0x7C, 0x00, 0x00, 0x00},
/* D (68) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0xC6, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC6, 0xFC, 0x00, 0x00, 0x00},
/* E (69) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0xC0, 0xC0, 0xC0, 0xF8, 0xC0, 0xC0, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* F (70) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0xC0, 0xC0, 0xC0, 0xF8, 0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00},
/* G (71) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0xC2, 0xC2, 0xC0, 0xCE, 0xC6, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* H (72) */ {0x00, 0x00, 0x00, 0x00, 0xC3, 0xC3, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* I (73) */ {0x00, 0x00, 0x00, 0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00},
/* J (74) */ {0x00, 0x00, 0x00, 0x00, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xF0, 0x00, 0x00, 0x00},
/* K (75) */ {0x00, 0x00, 0x00, 0x00, 0xC6, 0xCC, 0xD8, 0xF0, 0xE0, 0xF0, 0xD8, 0xCC, 0xC6, 0x00, 0x00, 0x00},
/* L (76) */ {0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* M (77) */ {0x00, 0x00, 0x00, 0x00, 0xC3, 0xE7, 0xE7, 0xFF, 0xDB, 0xC3, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* N (78) */ {0x00, 0x00, 0x00, 0x00, 0xE2, 0xE2, 0xB2, 0xB2, 0x9A, 0x9A, 0x8E, 0x8E, 0x86, 0x00, 0x00, 0x00},
/* O (79) */ {0x00, 0x00, 0x00, 0x00, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x00, 0x00, 0x00},
/* P (80) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0xC6, 0xC6, 0xC6, 0xC6, 0xFC, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00},
/* Q (81) */ {0x00, 0x00, 0x00, 0x00, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x06, 0x03, 0x00},
/* R (82) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0xC6, 0xC6, 0xC6, 0xFC, 0xD8, 0xCC, 0xC6, 0xC3, 0x00, 0x00, 0x00},
/* S (83) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0xC2, 0xC2, 0xC0, 0x7C, 0x06, 0x86, 0x86, 0x7C, 0x00, 0x00, 0x00},
/* T (84) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00},
/* U (85) */ {0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* V (86) */ {0x00, 0x00, 0x00, 0x00, 0xC3, 0xC3, 0xC3, 0x66, 0x66, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x00, 0x00},
/* W (87) */ {0x00, 0x00, 0x00, 0x00, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xDB, 0xFF, 0xE7, 0x42, 0x00, 0x00, 0x00},
/* X (88) */ {0x00, 0x00, 0x00, 0x00, 0xC3, 0xC3, 0x66, 0x3C, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Y (89) */ {0x00, 0x00, 0x00, 0x00, 0xC3, 0xC3, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00},
/* Z (90) */ {0x00, 0x00, 0x00, 0x00, 0xFC, 0x0C, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* [ (91) */ {0x00, 0x00, 0x00, 0x78, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x78, 0x00},
/* \ (92) */ {0x00, 0x00, 0x00, 0x80, 0x80, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x08, 0x08, 0x04, 0x04, 0x00},
/* ] (93) */ {0x00, 0x00, 0x00, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x78, 0x00},
/* ^ (94) */ {0x00, 0x00, 0x00, 0x00, 0x0C, 0x12, 0x21, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* _ (95) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00},
/* ` (96) */ {0x00, 0x00, 0x00, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
/* a (97) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* b (98) */ {0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xDC, 0xE6, 0xC6, 0xC6, 0xC6, 0xC6, 0xFC, 0x00, 0x00, 0x00},
/* c (99) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xC4, 0xC0, 0xC0, 0xC0, 0xC4, 0x78, 0x00, 0x00, 0x00},
/* d (100) */ {0x00, 0x00, 0x00, 0x06, 0x06, 0x06, 0x7E, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* e (101) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xFE, 0xC0, 0xC2, 0x7C, 0x00, 0x00, 0x00},
/* f (102) */ {0x00, 0x00, 0x00, 0x38, 0x60, 0x60, 0xF0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x00, 0x00, 0x00},
/* g (103) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x06, 0x86, 0x7C},
/* h (104) */ {0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xDC, 0xE6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00},
/* i (105) */ {0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00},
/* j (106) */ {0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0xE0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0xC0},
/* k (107) */ {0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xCC, 0xD8, 0xF0, 0xE0, 0xF0, 0xD8, 0xCC, 0x00, 0x00, 0x00},
/* l (108) */ {0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00},
/* m (109) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE6, 0xFF, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0x00, 0x00, 0x00},
/* n (110) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0xE6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00},
/* o (111) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* p (112) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0xE6, 0xC6, 0xC6, 0xC6, 0xC6, 0xFC, 0xC0, 0xC0, 0xC0},
/* q (113) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x06, 0x06, 0x06},
/* r (114) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD8, 0xF8, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00},
/* s (115) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xC4, 0xE0, 0x78, 0x1C, 0x8C, 0x78, 0x00, 0x00, 0x00},
/* t (116) */ {0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0xF8, 0x60, 0x60, 0x60, 0x60, 0x60, 0x38, 0x00, 0x00, 0x00},
/* u (117) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* v (118) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0x6C, 0x6C, 0x38, 0x38, 0x00, 0x00, 0x00},
/* w (119) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0xC3, 0xC3, 0xDB, 0xFF, 0x66, 0x24, 0x00, 0x00, 0x00},
/* x (120) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC, 0x78, 0x30, 0x78, 0xCC, 0xCC, 0x00, 0x00, 0x00},
/* y (121) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0x6C, 0x6C, 0x38, 0x38, 0x18, 0x30, 0x30},
/* z (122) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* { (123) */ {0x00, 0x00, 0x00, 0x1C, 0x30, 0x30, 0x30, 0x30, 0xE0, 0x30, 0x30, 0x30, 0x30, 0x30, 0x1C, 0x00},
/* | (124) */ {0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00},
/* } (125) */ {0x00, 0x00, 0x00, 0xE0, 0x30, 0x30, 0x30, 0x30, 0x1C, 0x30, 0x30, 0x30, 0x30, 0x30, 0xE0, 0x00},
/* ~ (126) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xFC, 0x8F, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00},
/* BLK (127) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* À (192) */ {0x00, 0x30, 0x18, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Á (193) */ {0x00, 0x06, 0x0C, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Â (194) */ {0x00, 0x3C, 0x66, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Ã (195) */ {0x00, 0x32, 0x4C, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Ä (196) */ {0x00, 0x66, 0x66, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Å (197) */ {0x00, 0x18, 0x24, 0x24, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Æ (198) */ {0x00, 0x00, 0x00, 0x00, 0x1F, 0x33, 0x33, 0x33, 0x63, 0x7F, 0x63, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* Ç (199) */ {0x00, 0x00, 0x00, 0x00, 0x7C, 0xC2, 0xC2, 0xC0, 0xC0, 0xC0, 0xC2, 0xC2, 0x7C, 0x08, 0x08, 0x30},
/* È (200) */ {0x00, 0x60, 0x30, 0x00, 0xFC, 0xC0, 0xC0, 0xC0, 0xF8, 0xC0, 0xC0, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* É (201) */ {0x00, 0x18, 0x30, 0x00, 0xFC, 0xC0, 0xC0, 0xC0, 0xF8, 0xC0, 0xC0, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* Ê (202) */ {0x00, 0x38, 0x6C, 0x00, 0xFC, 0xC0, 0xC0, 0xC0, 0xF8, 0xC0, 0xC0, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* Ë (203) */ {0x00, 0x6C, 0x6C, 0x00, 0xFC, 0xC0, 0xC0, 0xC0, 0xF8, 0xC0, 0xC0, 0xC0, 0xFC, 0x00, 0x00, 0x00},
/* Ì (204) */ {0x00, 0x18, 0x0C, 0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00},
/* Í (205) */ {0x00, 0x06, 0x0C, 0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00},
/* Î (206) */ {0x00, 0x71, 0xD9, 0x00, 0xF0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00},
/* Ï (207) */ {0x00, 0x98, 0x98, 0x00, 0xF0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00},
/* Ð (208) */ {0x00, 0x00, 0x00, 0x00, 0x7E, 0x63, 0x61, 0x61, 0xF9, 0x61, 0x61, 0x63, 0x7E, 0x00, 0x00, 0x00},
/* Ñ (209) */ {0x00, 0x32, 0x4C, 0x00, 0xE2, 0xE2, 0xB2, 0xB2, 0x9A, 0x9A, 0x8E, 0x8E, 0x86, 0x00, 0x00, 0x00},
/* Ò (210) */ {0x00, 0x30, 0x18, 0x00, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x00, 0x00, 0x00},
/* Ó (211) */ {0x00, 0x0C, 0x18, 0x00, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x00, 0x00, 0x00},
/* Ô (212) */ {0x00, 0x1C, 0x36, 0x00, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x00, 0x00, 0x00},
/* Õ (213) */ {0x00, 0x32, 0x4C, 0x00, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x00, 0x00, 0x00},
/* Ö (214) */ {0x00, 0x66, 0x66, 0x00, 0x7E, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x00, 0x00, 0x00},
/* × (215) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00},
/* Ø (216) */ {0x00, 0x00, 0x00, 0x01, 0x7E, 0xC7, 0xCB, 0xCB, 0xD3, 0xD3, 0xD3, 0xE3, 0x7E, 0x80, 0x00, 0x00},
/* Ù (217) */ {0x00, 0x30, 0x18, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* Ú (218) */ {0x00, 0x18, 0x30, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* Û (219) */ {0x00, 0x38, 0x6C, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* Ü (220) */ {0x00, 0x6C, 0x6C, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* Ý (221) */ {0x00, 0x0C, 0x18, 0x00, 0xC3, 0xC3, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00},
/* Þ (222) */ {0x00, 0x00, 0x00, 0x00, 0xC0, 0xFE, 0xC3, 0xC3, 0xC3, 0xC3, 0xFE, 0xC0, 0xC0, 0x00, 0x00, 0x00},
/* ß (223) */ {0x00, 0x00, 0x00, 0x78, 0xCC, 0xCC, 0xCC, 0xDC, 0xC6, 0xC6, 0xC6, 0xC6, 0xDC, 0x00, 0x00, 0x00},
/* à (224) */ {0x00, 0x00, 0x00, 0x30, 0x18, 0x00, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* á (225) */ {0x00, 0x00, 0x00, 0x0C, 0x18, 0x00, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* â (226) */ {0x00, 0x00, 0x00, 0x38, 0x6C, 0x00, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* ã (227) */ {0x00, 0x00, 0x00, 0x32, 0x4C, 0x00, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* ä (228) */ {0x00, 0x00, 0x00, 0x6C, 0x6C, 0x00, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* å (229) */ {0x00, 0x00, 0x18, 0x24, 0x24, 0x18, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* æ (230) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3B, 0x4C, 0x0C, 0x7F, 0xCC, 0xCC, 0x73, 0x00, 0x00, 0x00},
/* ç (231) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xC4, 0xC0, 0xC0, 0xC0, 0xC4, 0x78, 0x08, 0x08, 0x30},
/* è (232) */ {0x00, 0x00, 0x00, 0x60, 0x30, 0x00, 0x7C, 0xC6, 0xC6, 0xFE, 0xC0, 0xC2, 0x7C, 0x00, 0x00, 0x00},
/* é (233) */ {0x00, 0x00, 0x00, 0x0C, 0x18, 0x00, 0x7C, 0xC6, 0xC6, 0xFE, 0xC0, 0xC2, 0x7C, 0x00, 0x00, 0x00},
/* ê (234) */ {0x00, 0x00, 0x00, 0x38, 0x6C, 0x00, 0x7C, 0xC6, 0xC6, 0xFE, 0xC0, 0xC2, 0x7C, 0x00, 0x00, 0x00},
/* ë (235) */ {0x00, 0x00, 0x00, 0x6D, 0x6C, 0x00, 0x7C, 0xC6, 0xC6, 0xFE, 0xC0, 0xC2, 0x7C, 0x00, 0x00, 0x00},
/* ì (236) */ {0x00, 0x00, 0x00, 0x30, 0x18, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00},
/* í (237) */ {0x00, 0x00, 0x00, 0x0C, 0x18, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00},
/* î (238) */ {0x00, 0x00, 0x00, 0xC3, 0x23, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00},
/* ï (239) */ {0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00},
/* ð (240) */ {0x00, 0x00, 0x00, 0x34, 0x18, 0x2C, 0x06, 0x7E, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* ñ (241) */ {0x00, 0x00, 0x00, 0x32, 0x4C, 0x00, 0xDC, 0xE6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00},
/* ò (242) */ {0x00, 0x00, 0x00, 0x60, 0x30, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* ó (243) */ {0x00, 0x00, 0x00, 0x0C, 0x18, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* ô (244) */ {0x00, 0x00, 0x00, 0x38, 0x6C, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* õ (245) */ {0x00, 0x00, 0x00, 0x32, 0x4C, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* ö (246) */ {0x00, 0x00, 0x00, 0x6C, 0x6C, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* ÷ (247) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00, 0x7F, 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00},
/* ø (248) */ {0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7C, 0xCE, 0xCE, 0xD6, 0xE6, 0xE6, 0x7C, 0x80, 0x00, 0x00},
/* ù (249) */ {0x00, 0x00, 0x00, 0x60, 0x30, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* ú (250) */ {0x00, 0x00, 0x00, 0x0C, 0x18, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* û (251) */ {0x00, 0x00, 0x00, 0x38, 0x6C, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* ü (252) */ {0x00, 0x00, 0x00, 0x6C, 0x6C, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* ý (253) */ {0x00, 0x00, 0x00, 0x0C, 0x18, 0x00, 0xC6, 0xC6, 0xC6, 0x6C, 0x6C, 0x38, 0x38, 0x18, 0x30, 0x30},
/* þ (254) */ {0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xDC, 0xE6, 0xC6, 0xC6, 0xC6, 0xC6, 0xFC, 0xC0, 0xC0, 0xC0},
/* ÿ (255) */ {0x00, 0x00, 0x00, 0x6C, 0x6C, 0x00, 0xC6, 0xC6, 0xC6, 0x6C, 0x6C, 0x38, 0x38, 0x18, 0x30, 0x30},
/* Ā (256) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* ā (257) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* Ă (258) */ {0x00, 0x22, 0x1C, 0x00, 0x18, 0x3C, 0x66, 0xC3, 0xC3, 0xFF, 0xC3, 0xC3, 0xC3, 0x00, 0x00, 0x00},
/* ă (259) */ {0x00, 0x00, 0x00, 0x22, 0x1C, 0x00, 0x3C, 0x46, 0x06, 0x7E, 0xC6, 0xC6, 0x7E, 0x00, 0x00, 0x00},
/* Ą (260) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* ą (261) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* Ć (262) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* ć (263) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* Ĉ (264) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* ĉ (265) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* Ċ (266) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* ċ (267) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* Č (268) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* č (269) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* Ď (270) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* ď (271) */ {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
/* Đ (272) */ {0x00, 0x00, 0x00, 0x00, 0x7E, 0x63, 0x61, 0x61, 0xF9, 0x61, 0x61, 0x63, 0x7E, 0x00, 0x00, 0x00},
/* đ (273) */ {0x00, 0x00, 0x00, 0x06, 0x1F, 0x06, 0x7E, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* Ĩ (296) */ {0x00, 0x19, 0x26, 0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00},
/* ĩ (297) */ {0x00, 0x00, 0x00, 0x34, 0x2C, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00},
/* Ũ (360) */ {0x00, 0x32, 0x4C, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* ũ (361) */ {0x00, 0x00, 0x00, 0x32, 0x4C, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
/* Ơ (416) */ {0x00, 0x00, 0x03, 0x01, 0x7F, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x7E, 0x00, 0x00, 0x00},
/* ơ (417) */ {0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x7E, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* Ư (431) */ {0x00, 0x00, 0x03, 0x01, 0xC7, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00},
/* ư (432) */ {0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0xC7, 0xC6, 0xC6, 0xC6, 0xC6, 0xCE, 0x76, 0x00, 0x00, 0x00},
};
#endif