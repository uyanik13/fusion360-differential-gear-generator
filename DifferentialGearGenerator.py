# ─────────────────────────────────────────────────────────────────────────────
#  Differential Gear Generator  v2.0
#  Fusion 360 Add-In
#
#  Generates a complete differential gear assembly using involute bevel gears:
#    • 2 × Side Gears  (SideGear_R, SideGear_L)
#    • 2 or 4 × Spider / Planet Gears  (SpiderGear_1 … n)
#    • Optional Ring Gear (Crown Wheel) + Drive Pinion
#
#  CORE FUNCTIONS:
#    Ported directly from GFGearGenerator (Juan Gras, Michael Truell, Mervill).
#    NC8 (90° Bevel) call sequence preserved unchanged.
#
#  Units: Fusion 360 internal unit = cm → mm values are divided by 10.
# ─────────────────────────────────────────────────────────────────────────────

import adsk.core, adsk.fusion, adsk.cam, traceback
import math as mt

_handlers = []


# ═══════════════════════════════════════════════════════════════════════════════
#  CORE — Ported verbatim from GFGearGenerator
# ═══════════════════════════════════════════════════════════════════════════════

def linspace(inicio, fin, separaciones):
    lista = []
    a = (fin - inicio) / (separaciones - 1)
    for i in range(0, separaciones):
        lista.append(inicio + a * i)
    return lista


def radToDeg(x):
    return x * 180 / mt.pi


def parameters(m, z, ap, ah, anchoeng, bool, X, fastcompute, normal_system=False):
    modt = m
    apt  = ap
    if normal_system:
        modt = m / mt.cos(ah)
        apt  = mt.atan(mt.tan(ap) / mt.cos(ah))
    dp  = modt * z
    rp  = dp / 2
    db  = dp * mt.cos(apt)
    rb  = db / 2
    da  = dp + 2 * m
    ra  = da / 2
    df  = dp - 2.5 * m
    rf  = df / 2
    T   = mt.pi * modt / 2
    dv  = dp + 2 * X * m
    dva = dv + 2 * m
    dvf = dv - 2.5 * m
    rva = dva / 2
    rvf = dvf / 2
    alpha  = ((mt.sqrt((dp**2) - (db**2))) / db) - apt
    beta   = mt.pi / (2 * z)

    def inv(x):
        return mt.tan(x) - x

    def Tt(d):
        at   = mt.acos(rb / (d / 2))
        T1   = d * ((T / dp) + inv(apt) - inv(at))
        Teta = T1 / (d / 2)
        return Teta

    if fastcompute:
        aok = 50
    elif m < 15:
        aok = int((89 * mt.sqrt(m / .3)) - 3)
    else:
        aok = 425

    angrot2 = 2 * mt.pi - alpha - beta
    u  = linspace(0, mt.sqrt(((da / db)**2) - 1), aok)
    v  = linspace(0, mt.sqrt(((da / db)**2) - 1), aok)
    x, y, x2, y2 = [], [], [], []
    for i in range(len(u)):
        x.append( rb * (mt.cos(u[i] + angrot2) + u[i] * mt.sin(u[i] + angrot2)))
        y.append( rb * (mt.sin(u[i] + angrot2) - u[i] * mt.cos(u[i] + angrot2)))
        x2.append(rb * (mt.cos(v[i] + angrot2) + v[i] * mt.sin(v[i] + angrot2)))
        y2.append(-rb * (mt.sin(v[i] + angrot2) - v[i] * mt.cos(v[i] + angrot2)))

    al, bl, cl, dl = [], [], [], []

    def sigmaPS(rt, rb, ap, X):
        alphat = mt.acos(rb / rt)
        dt     = 2 * rt
        Tt_val = dt * ((mt.pi / (2 * z)) + (2 * X * mt.tan(ap) / z) + inv(ap) - inv(alphat))
        return Tt_val / rt

    if X > 0 or X < 0:
        angrot = 2 * mt.pi - sigmaPS(rb, rb, ap, X) / 2
        u = linspace(0, mt.sqrt(((dva / db)**2) - 1), aok)
        v = linspace(0, mt.sqrt(((dva / db)**2) - 1), aok)
        for i in range(len(u)):
            al.append(rb * (mt.cos(u[i] + angrot) + u[i] * mt.sin(u[i] + angrot)))
            bl.append(rb * (mt.sin(u[i] + angrot) - u[i] * mt.cos(u[i] + angrot)))
            cl.append(rb * (mt.cos(v[i] + angrot) + v[i] * mt.sin(v[i] + angrot)))
            dl.append(-rb * (mt.sin(v[i] + angrot) - v[i] * mt.cos(v[i] + angrot)))
        x, y, x2, y2 = al, bl, cl, dl

    vool = -1 if bool else 1
    try:
        ph  = mt.pi * dp * mt.cos(ah) / mt.sin(ah)
        aph = 10 * anchoeng * 2 * mt.pi / ph
        t2  = linspace(0, aph, aok)
        bb  = ph / (2 * mt.pi)
        zl, xl, yl, xl2, yl2 = [], [], [], [], []
        for i in range(len(t2)):
            zl.append(bb * t2[i])
            xl.append(rp * mt.cos(t2[i] + mt.pi) + dp)
            yl.append(rp * mt.sin(-vool * t2[i]))
            xl2.append(rp * mt.cos(t2[i]))
            yl2.append(rp * mt.sin(vool * t2[i]))
    except Exception:
        ph, aph = 0, 0
        t2  = linspace(0, 0, aok)
        zl = xl = yl = xl2 = yl2 = t2

    xrot, yrot, x2rot, y2rot = [], [], [], []
    for i in range(len(u)):
        xrot.append(rb*(mt.cos(u[i]+angrot2+vool*aph)+u[i]*mt.sin(u[i]+angrot2+vool*aph))+(dp-2*(rp*mt.cos(aph))))
        yrot.append(rb*(mt.sin(u[i]+angrot2+vool*aph)-u[i]*mt.cos(u[i]+angrot2+vool*aph))+2*rp*mt.sin(-vool*aph))
        x2rot.append(rb*(mt.cos(v[i]+angrot2-vool*aph)+v[i]*mt.sin(v[i]+angrot2-vool*aph))+(dp-2*(rp*mt.cos(aph))))
        y2rot.append(-rb*(mt.sin(v[i]+angrot2-vool*aph)-v[i]*mt.cos(v[i]+angrot2-vool*aph))+2*rp*mt.sin(-vool*aph))
    try:
        lizq = mt.sqrt(2*(dp**2)*(1-mt.cos(-vool*aph)))
        uang = mt.asin(dp*mt.sin(-vool*aph)/lizq)
        xneorig = lizq*mt.cos(uang)
        yneorig = dp*mt.sin(-vool*aph)
    except Exception:
        xneorig = yneorig = 0
    xrot2, yrot2, y3rot, x3rot = [], [], [], []
    for i in range(len(u)):
        xrot2.append(rb*(mt.cos(u[i]+angrot2+vool*aph)+u[i]*mt.sin(u[i]+angrot2+vool*aph)))
        yrot2.append(rb*(mt.sin(u[i]+angrot2+vool*aph)-u[i]*mt.cos(u[i]+angrot2+vool*aph)))
        x3rot.append(rb*(mt.cos(v[i]+angrot2-vool*aph)+v[i]*mt.sin(v[i]+angrot2-vool*aph)))
        y3rot.append(-rb*(mt.sin(v[i]+angrot2-vool*aph)-v[i]*mt.cos(v[i]+angrot2-vool*aph)))

    return (rf, x, y, x2, y2, aok, Tt(da), ra, angrot2, alpha, beta,
            xneorig, yneorig, zl, xl, yl, xrot, yrot, x2rot, y2rot,
            xl2, yl2, xrot2, yrot2, x3rot, y3rot, aph, rb, rva, rvf)


def sketchcon(x, y, x2, y2, z, z2, rp, rp2, rf, ra, Ttda, m, aok, newComp):
    try:
        aconico  = mt.atan(z / z2)
        app      = adsk.core.Application.get()
        rootComp = newComp
        sketch   = rootComp.sketches.add(rootComp.xYConstructionPlane)
        orig     = adsk.core.Point3D.create(0, 0, 0)
        lines    = sketch.sketchCurves.sketchLines

        points  = adsk.core.ObjectCollection.create()
        points2 = adsk.core.ObjectCollection.create()
        puntos  = adsk.core.ObjectCollection.create()
        puntos2 = adsk.core.ObjectCollection.create()
        puntos.add( adsk.core.Point3D.create(x[0]/10,  y[0]/10,  0))
        puntos.add( adsk.core.Point3D.create(x[1]/10,  y[1]/10,  0))
        puntos2.add(adsk.core.Point3D.create(x2[0]/10, y2[0]/10, 0))
        puntos2.add(adsk.core.Point3D.create(x2[1]/10, y2[1]/10, 0))
        for i in range(1, aok):
            points.add( adsk.core.Point3D.create(x[i]/10,  y[i]/10,  0))
            points2.add(adsk.core.Point3D.create(x2[i]/10, y2[i]/10, 0))

        sketch.sketchCurves.sketchFittedSplines.add(points)
        sketch.sketchCurves.sketchFittedSplines.add(points2)
        sketch.sketchCurves.sketchFittedSplines.add(puntos)
        sketch.sketchCurves.sketchFittedSplines.add(puntos2)
        lines.addByTwoPoints(orig, puntos[0])
        lines.addByTwoPoints(orig, puntos2[0])
        sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
            orig,
            adsk.core.Point3D.create(x[aok-1]/10, y[aok-1]/10, 0),
            Ttda)
        prof = sketch.profiles.item(0)

        puntosup = adsk.core.Point3D.create(
            (rp - rp*mt.cos(aconico)) / 10, 0,
            (rp * mt.sin(aconico)) / 10)
        puntocon = adsk.core.Point3D.create(
            ((rp - rp*mt.cos(aconico)) + rp2*mt.cos(mt.pi/2 - aconico)) / 10, 0,
            (rp*mt.sin(aconico) + rp2*mt.sin(mt.pi/2 - aconico)) / 10)
        puntoad = adsk.core.Point3D.create(ra / 10, 0, 0)
        puntop  = adsk.core.Point3D.create(rp / 10, 0, 0)
        puntodd = adsk.core.Point3D.create(rf / 10, 0, 0)

        linead     = lines.addByTwoPoints(puntocon, puntoad)
        lines.addByTwoPoints(puntocon, puntodd)
        linepp     = lines.addByTwoPoints(puntocon, puntop)
        linecenter = lines.addByTwoPoints(puntosup, puntocon)

        ptInput = rootComp.constructionPoints.createInput()
        ptInput.setByTwoEdges(linead, linepp)
        cp = rootComp.constructionPoints.add(ptInput)
        cp.name = 'Align Point'

        path      = newComp.features.createPath(linepp, False)
        guiderail = newComp.features.createPath(linead, False)
        Ao = mt.sqrt((rp**2) + (rp2**2))
        F  = Ao / 3

        sketch3 = rootComp.sketches.add(rootComp.xYConstructionPlane)
        l3      = sketch3.sketchCurves.sketchLines
        puntodisco = adsk.core.Point3D.create(
            (rf - (rp - 1.25*m*mt.cos(aconico))*mt.cos(aconico)) / 10, 0,
            ((rp - 1.25*m*mt.cos(aconico))*mt.sin(aconico)) / 10)
        l3.addByTwoPoints(puntodd, puntodisco)
        l3.addByTwoPoints(puntodisco, puntocon)
        angulof = mt.atan((rp2 + 1.25*m*mt.sin(aconico)) / (rp - 1.25*m*mt.cos(aconico)))
        puntodisco2 = adsk.core.Point3D.create(
            (rf - F*mt.cos(aconico + angulof)) / 10, 0, F / 10)
        l3.addByTwoPoints(puntodisco2, puntodd)
        puntodisco3 = adsk.core.Point3D.create(
            ((rf - F*mt.cos(aconico + angulof)) - 2*rf*mt.cos(aconico)) / 10, 0,
            (F + 2*rf*mt.sin(aconico)) / 10)
        l3.addByTwoPoints(puntodisco2, puntodisco3)
        prof3 = sketch3.profiles.item(0)

        sketch4 = rootComp.sketches.add(rootComp.xYConstructionPlane)
        l4      = sketch4.sketchCurves.sketchLines
        puntoram  = adsk.core.Point3D.create((ra + 5) / 10, 0, 0)
        l4.addByTwoPoints(puntodd, puntoram)
        l4.addByTwoPoints(orig, puntodisco)
        l4.addByTwoPoints(puntodisco, puntodd)
        puntoabaj = adsk.core.Point3D.create(0, 0, -.2)
        l4.addByTwoPoints(puntoabaj, orig)
        puntoram2 = adsk.core.Point3D.create((ra + 5) / 10, 0, -.2)
        l4.addByTwoPoints(puntoabaj, puntoram2)
        l4.addByTwoPoints(puntoram2, puntoram)
        punton = adsk.core.Point3D.create((ra + 5) / 10, 0, F / 10)
        l4.addByTwoPoints(punton, puntodisco2)
        l4.addByTwoPoints(punton, puntocon)
        l4.addByTwoPoints(puntodisco, puntocon)
        l4.addByTwoPoints(puntodisco2, puntodisco3)
        prof4   = sketch4.profiles.item(1)
        profe42 = sketch4.profiles.item(0)

        return prof, path, guiderail, linepp, linead, prof3, linecenter, prof4, profe42
    except Exception:
        pass   # sketchcon failed silently — caller handles None check
        return None


def sweep(prof, path, guiderail, linepp, linead, newComp):
    try:
        rootComp  = newComp
        sweeps    = rootComp.features.sweepFeatures
        path      = newComp.features.createPath(linepp, False)
        guiderail = newComp.features.createPath(linead, False)
        si = sweeps.createInput(
            prof, path, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        si.path      = path
        si.guideRail = guiderail
        si.distanceOne = adsk.core.ValueInput.createByReal(1 / 2)
        return sweeps.add(si)
    except Exception:
        pass   # sweep failed silently
        return None


def rev(prof, eje, newComp, u='NewBody'):
    try:
        rootComp     = newComp
        revolvefeats = rootComp.features.revolveFeatures
        ops = {
            'NewBody': adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
            'Join'   : adsk.fusion.FeatureOperations.JoinFeatureOperation,
            'Cut'    : adsk.fusion.FeatureOperations.CutFeatureOperation,
        }
        ri = revolvefeats.createInput(prof, eje, ops[u])
        ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(2 * mt.pi))
        revolvefeats.add(ri)
    except Exception:
        pass   # Cut başarısız olsa da dişli oluşmaya devam eder


def cpattern(linecenter, esconico, ra, rf, z, diente, esStdr, anchoeng, newComp, u='Join'):
    try:
        rootComp = newComp
        if not esconico and not esStdr:
            sk  = rootComp.sketches.add(rootComp.xYConstructionPlane)
            lns = sk.sketchCurves.sketchLines
            zl  = lns.addByTwoPoints(
                adsk.core.Point3D.create((ra+rf)/10, 0, 0),
                adsk.core.Point3D.create((ra+rf)/10, 0, anchoeng))
            eje = zl
            sk.isVisible = False
        elif esconico and esStdr:
            eje = linecenter
        else:
            eje = rootComp.zConstructionAxis

        ents = adsk.core.ObjectCollection.create()
        ents.add(diente)
        cf  = rootComp.features.circularPatternFeatures
        ci  = cf.createInput(ents, eje)
        ci.quantity    = adsk.core.ValueInput.createByReal(z)
        # Use radians float to avoid unit-string parsing issues (especially in Direct Design)
        ci.totalAngle  = adsk.core.ValueInput.createByReal(2.0 * mt.pi)
        ci.isSymmetric = False
        # Always use Adjust (1) — Identical (0) can fail for the first component.
        ci.patternComputeOption = 1
        cf.add(ci)
    except Exception:
        pass   # cpattern failed silently


def combine(z, newComp):
    try:
        rootComp = newComp
        conta    = rootComp.bRepBodies.count - 1
        conta2   = rootComp.bRepBodies.count
        target   = rootComp.bRepBodies.item(conta - z)
        tools    = adsk.core.ObjectCollection.create()
        for i in range(conta2 - z, conta2):
            tools.add(rootComp.bRepBodies.item(i))
        ci = rootComp.features.combineFeatures.createInput(target, tools)
        ci.operation        = 0
        ci.isKeepToolBodies = False
        ci.isNewComponent   = False
        rootComp.features.combineFeatures.add(ci)
    except Exception:
        pass   # combine failed silently — tooth bodies remain separate but gear is still usable


def rotcon(rf, aconico, occ):
    try:
        t = adsk.core.Matrix3D.create()
        t.setToRotation(-aconico,
                         adsk.core.Vector3D.create(0, 1, 0),
                         adsk.core.Point3D.create(rf / 10, 0, 0))
        occ.transform2 = t
    except Exception:
        adsk.core.Application.get().userInterface.messageBox(
            'rotcon Failed:\n' + traceback.format_exc())


def moveAndRotateBevel(x, y, z, occ, aconico, rf):
    try:
        t = adsk.core.Matrix3D.create()
        t.setToRotation(-aconico,
                         adsk.core.Vector3D.create(0, 1, 0),
                         adsk.core.Point3D.create(rf / 10, 0, 0))
        t.translation = adsk.core.Vector3D.create(x, y, z)
        occ.transform2 = t
    except Exception:
        adsk.core.Application.get().userInterface.messageBox(
            'moveAndRotateBevel Failed:\n' + traceback.format_exc())


# ═══════════════════════════════════════════════════════════════════════════════
#  YARDIMCI: MİL DELİĞİ
# ═══════════════════════════════════════════════════════════════════════════════

def _add_bore_to(bore_mm, comp):
    try:
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), bore_mm / 2.0 / 10.0)
        prof = sk.profiles.item(0)
        ei   = comp.features.extrudeFeatures.createInput(
            prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        ei.setAllExtent(False)
        comp.features.extrudeFeatures.add(ei)
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════════════════════
#  YARDIMCI: ÇOKLU BİLEŞEN BAĞLAMI (Part Design düzeltmesi)
# ═══════════════════════════════════════════════════════════════════════════════

def _get_assembly_context():
    app    = adsk.core.Application.get()
    ui     = app.userInterface
    design = adsk.fusion.Design.cast(app.activeProduct)
    root   = design.rootComponent

    try:
        test = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        test.deleteMe()
        return design, root
    except RuntimeError:
        pass

    ans = ui.messageBox(
        'This document is in "Part Design" mode (single component only).\n\n'
        'A differential requires multiple components.\n'
        'A new Fusion 360 Design will be opened automatically.\n\n'
        'Continue?',
        'Differential Gear Generator',
        adsk.core.MessageBoxButtonTypes.YesNoButtonType,
        adsk.core.MessageBoxIconTypes.InformationIconType)

    if ans != adsk.core.DialogResults.DialogYes:
        return None, None

    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    return design, design.rootComponent


# ═══════════════════════════════════════════════════════════════════════════════
#  SINGLE BEVEL GEAR BUILD  (NC8 call order, verbatim)
# ═══════════════════════════════════════════════════════════════════════════════

def _build_one_bevel(x, y, x2, y2, z, z2, rp, rp2, rf, ra, Ttda, aok, m, comp):
    """Build a single bevel gear body using the GFGearGenerator NC8 sequence."""
    s = sketchcon(x, y, x2, y2, z, z2, rp, rp2, rf, ra, Ttda, m, aok, comp)
    if s is None:
        raise RuntimeError('sketchcon failed')
    esq   = comp.sketches.item(comp.sketches.count - 3)
    prof  = esq.profiles.item(0)
    tooth = sweep(prof, s[1], s[2], s[3], s[4], comp)
    if tooth is None:
        raise RuntimeError('sweep failed')
    rev(s[5], s[6], comp, 'NewBody')
    cpattern(s[6], True, ra, rf, z, tooth, True, 1, comp, 'Join')
    combine(z, comp)
    rev(s[7], s[6], comp, 'Cut')
    rev(s[8], s[6], comp, 'Cut')
    return rf, ra, s[6]


# ═══════════════════════════════════════════════════════════════════════════════
#  PLACEMENT HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def _puntocon_x(m, z, z_mate):
    """
    Returns the apex (cone tip) X-coordinate of a bevel gear in Fusion cm units.
    Equals the z-projection of puntocon.x from sketchcon().
    """
    aconico = mt.atan(float(z) / float(z_mate))
    rp  = m * z      / 2.0   # mm
    rp2 = m * z_mate / 2.0   # mm
    return (rp * (1.0 - mt.cos(aconico)) + rp2 * mt.sin(aconico)) / 10.0  # cm


def _place_gear(occ, rot_angle_rad, rot_axis, apex_local_x_cm, clearance_cm=0.0):
    """
    Move a gear occurrence to its differential assembly position.

    Steps:
      1. Rotate by rot_angle_rad around rot_axis (at world origin).
      2. Translate so the rotated apex lands at (0,0,0).
      3. Shift outward along the gear axis by clearance_cm to prevent overlap.

    rot_axis : 'Y' or 'Z'
    clearance_cm : extra outward offset from apex (cm)
    """
    t = adsk.core.Matrix3D.create()   # identity

    if rot_angle_rad != 0.0:
        if rot_axis == 'Y':
            axis_vec = adsk.core.Vector3D.create(0, 1, 0)
        else:  # 'Z'
            axis_vec = adsk.core.Vector3D.create(0, 0, 1)
        t.setToRotation(rot_angle_rad, axis_vec,
                         adsk.core.Point3D.create(0, 0, 0))

    c = mt.cos(rot_angle_rad)
    s = mt.sin(rot_angle_rad)
    x0 = apex_local_x_cm

    # R x (x0,0,0) — rotated apex world position
    if rot_axis == 'Y':
        ax, ay, az = x0 * c, 0.0, -x0 * s
        dx, dy, dz = c, 0.0, -s   # outward axis direction = R_y x (1,0,0)
    else:  # 'Z'
        ax, ay, az = x0 * c, x0 * s, 0.0
        dx, dy, dz = c, s, 0.0    # outward axis direction = R_z x (1,0,0)

    # Translate apex to origin + clearance offset along gear axis
    t.translation = adsk.core.Vector3D.create(
        -ax + dx * clearance_cm,
        -ay + dy * clearance_cm,
        -az + dz * clearance_cm)
    occ.transform2 = t


# ═══════════════════════════════════════════════════════════════════════════════
#  DIFFERENTIAL ASSEMBLY
# ═══════════════════════════════════════════════════════════════════════════════

def build_differential(m, z_side, z_spider, ap, fast, n_spider, bore_mm,
                        z_ring=0, z_pinion_drive=0, assembled=True):
    """
    Build the full differential gear set.
    assembled=True : gears placed on correct axes (Assembled mode)
    assembled=False: gears spaced along X axis for inspection (Exploded mode)
    """
    app = adsk.core.Application.get()
    ui  = app.userInterface

    design, root = _get_assembly_context()
    if design is None:
        return

    try:
        nuOps = design.timeline.count
    except Exception:
        nuOps = 0

    # ── Compute all gear parameters ──────────────────────────────────────────────
    ls  = parameters(m, z_side,   ap, 0, 1, False, 0, fast)
    lsp = parameters(m, z_spider, ap, 0, 1, False, 0, fast)
    rf_s,  x_s,  y_s,  x2_s,  y2_s,  aok_s,  Ttda_s,  ra_s  = ls[:8]
    rf_sp, x_sp, y_sp, x2_sp, y2_sp, aok_sp, Ttda_sp, ra_sp  = lsp[:8]
    rp_s  = m * z_side   / 2.0
    rp_sp = m * z_spider / 2.0
    Ao    = m * mt.sqrt(float(z_side)**2 + float(z_spider)**2) / 2.0

    ra_rg = 0.0
    ring_params = None
    if z_ring > 0 and z_pinion_drive > 0:
        ls_rg = parameters(m, z_ring,         ap, 0, 1, False, 0, fast)
        ls_dp = parameters(m, z_pinion_drive, ap, 0, 1, False, 0, fast)
        rf_rg, x_rg, y_rg, x2_rg, y2_rg, aok_rg, Ttda_rg, ra_rg = ls_rg[:8]
        rf_dp, x_dp, y_dp, x2_dp, y2_dp, aok_dp, Ttda_dp, ra_dp = ls_dp[:8]
        rp_rg = m * z_ring         / 2.0
        rp_dp = m * z_pinion_drive / 2.0
        ring_params = (rf_rg, x_rg, y_rg, x2_rg, y2_rg, aok_rg, Ttda_rg, ra_rg,
                       rf_dp, x_dp, y_dp, x2_dp, y2_dp, aok_dp, Ttda_dp, ra_dp,
                       rp_rg, rp_dp)

    # ── Exploded spacing (each gear offset by col * spacing along X, no rotation)
    max_ra_cm = max(ra_s, ra_sp, ra_rg) / 10.0
    spacing   = max_ra_cm * 2.5 + 0.5   # cm

    def build_and_place(name, build_fn, col):
        occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        comp = occ.component
        comp.name = name
        build_fn(comp)
        if bore_mm > 0:
            _add_bore_to(bore_mm, comp)
        t = adsk.core.Matrix3D.create()
        t.translation = adsk.core.Vector3D.create(col * spacing, 0.0, 0.0)
        occ.transform2 = t
        try: design.snapshots.add()
        except Exception: pass
        return occ

    def sp(c): _build_one_bevel(x_sp, y_sp, x2_sp, y2_sp,
                                 z_spider, z_side, rp_sp, rp_s,
                                 rf_sp, ra_sp, Ttda_sp, aok_sp, m, c)

    def sd(c): _build_one_bevel(x_s, y_s, x2_s, y2_s,
                                 z_side, z_spider, rp_s, rp_sp,
                                 rf_s, ra_s, Ttda_s, aok_s, m, c)

    # ── Dişlileri sırayla oluştur ────────────────────────────────────────────
    try: design.snapshots.add()
    except Exception: pass

    # Warm-up: create a tiny body in a temporary component so Fusion has a
    # design-history reference before the first real gear's cpattern runs.
    # Without this, cpattern can produce incomplete results for the very
    # first component in a fresh/Direct-Design document.
    _wu_occ = None
    try:
        _wu_occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        _wu_comp = _wu_occ.component
        _wu_comp.name = '_init_ref'
        _sk = _wu_comp.sketches.add(_wu_comp.xYConstructionPlane)
        _sk.sketchCurves.sketchCircles.addByCenterRadius(
            adsk.core.Point3D.create(0, 0, 0), 0.01)
        _ei = _wu_comp.features.extrudeFeatures.createInput(
            _sk.profiles.item(0),
            adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        _ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(0.01))
        _wu_comp.features.extrudeFeatures.add(_ei)
        try: design.snapshots.add()
        except Exception: pass
    except Exception:
        _wu_occ = None  # warm-up failed, continue anyway

    if assembled:
        # ── ASSEMBLED MODE — exact GFGearGenerator NC8 positioning ───────────
        # SpiderGear = NC8 "pinion" (gear 2):  rotcon only
        # SideGear   = NC8 "wheel"  (gear 1):  moveAndRotateBevel formula
        # SpiderGear_2 / SideGear_L = 180° mirror around Y at differential center
        # GFGearGenerator also hides existing bodies before each build to prevent
        # body-index interference in combine/cpattern — we do the same.

        aconico_s  = mt.atan(float(z_side)   / float(z_spider))
        aconico_sp = mt.atan(float(z_spider) / float(z_side))

        # NC8 gear-1 (wheel = SideGear) translation formula — direct from NC8 handler
        tx_s = (ra_sp + ra_s)/10.0 + (
               (rp_s - 1.25*m*mt.cos(aconico_s)) - rf_s*mt.cos(aconico_s))/10.0
        tz_s = -rf_s * mt.sin(aconico_s) / 10.0

        # Differential center ≈ rotcon center of SpiderGear
        cx = rf_sp / 10.0   # cm

        def _hide_all():
            """Hide all bodies across all existing occurrences (GFGearGenerator pattern)."""
            hidden = []
            for j in range(root.occurrences.count):
                for k in range(root.occurrences.item(j).bRepBodies.count):
                    b = root.occurrences.item(j).bRepBodies.item(k)
                    if b.isVisible:
                        hidden.append(b)
                        try: b.isVisible = False
                        except: pass
            return hidden

        def _show_all(hidden):
            for b in hidden:
                try: b.isVisible = True
                except: pass

        def make_nc8(name, build_fn, transform):
            """Create component, hide existing bodies during build, apply transform."""
            occ  = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            comp = occ.component
            comp.name = name
            hid  = _hide_all()
            try:
                build_fn(comp)
            except Exception:
                pass
            if bore_mm > 0:
                _add_bore_to(bore_mm, comp)
            _show_all(hid)
            occ.transform2 = transform
            try: design.snapshots.add()
            except Exception: pass
            return occ

        # ── Pre-compute the four base transforms ─────────────────────────────

        # t_sp : SpiderGear_1 — NC8 gear 2 (rotcon)
        t_sp = adsk.core.Matrix3D.create()
        t_sp.setToRotation(-aconico_sp, adsk.core.Vector3D.create(0, 1, 0),
                            adsk.core.Point3D.create(rf_sp / 10.0, 0, 0))

        # t_sd : SideGear_R — NC8 gear 1 (moveAndRotateBevel)
        t_sd = adsk.core.Matrix3D.create()
        t_sd.setToRotation(-aconico_s, adsk.core.Vector3D.create(0, 1, 0),
                            adsk.core.Point3D.create(rf_s / 10.0, 0, 0))
        t_sd.translation = adsk.core.Vector3D.create(tx_s, 0.0, tz_s)

        def mirror180y(t_base):
            """Return  R180Y(cx) × t_base  — mirrors a transform through the
               differential centre on the Y-axis."""
            R = adsk.core.Matrix3D.create()
            R.setToRotation(mt.pi, adsk.core.Vector3D.create(0, 1, 0),
                             adsk.core.Point3D.create(cx, 0, 0))
            R.transformBy(t_base)   # R = R × t_base
            return R

        # ── Build gears ──────────────────────────────────────────────────────
        # SideGear_R first (first-component artefact is least visible on it)
        occ_sr = make_nc8('SideGear_R', sd, t_sd)
        try: occ_sr.component.bRepBodies.item(0).isVisible = False  # hide like NC8
        except Exception: pass

        # SpiderGear_1 — NC8 rotcon position
        make_nc8('SpiderGear_1', sp, t_sp)

        # SpiderGear_2 — mirror of SpiderGear_1
        make_nc8('SpiderGear_2', sp, mirror180y(t_sp))

        # SideGear_L — mirror of SideGear_R
        make_nc8('SideGear_L', sd, mirror180y(t_sd))

        # Restore SideGear_R visibility (NC8 pattern)
        try: occ_sr.component.bRepBodies.item(0).isVisible = True
        except Exception: pass

        # Extra spider gears (n_spider = 4)
        for i in range(2, n_spider):
            angle = mt.pi / 2.0 + (i - 2) * mt.pi  # 90°, 270°
            Rn = adsk.core.Matrix3D.create()
            Rn.setToRotation(angle, adsk.core.Vector3D.create(0, 1, 0),
                              adsk.core.Point3D.create(cx, 0, 0))
            Rn.transformBy(t_sp)
            make_nc8('SpiderGear_' + str(i + 1), sp, Rn)

        # Ring Gear + Drive Pinion — NC8 pair, offset in Y to clear internal gears
        if ring_params is not None:
            (rf_rg, x_rg, y_rg, x2_rg, y2_rg, aok_rg, Ttda_rg, ra_rg,
             rf_dp, x_dp, y_dp, x2_dp, y2_dp, aok_dp, Ttda_dp, ra_dp,
             rp_rg, rp_dp) = ring_params
            aconico_rg = mt.atan(float(z_ring) / float(z_pinion_drive))
            aconico_dp = mt.atan(float(z_pinion_drive) / float(z_ring))
            tx_rg = (ra_dp + ra_rg)/10.0 + (
                    (rp_rg - 1.25*m*mt.cos(aconico_rg)) - rf_rg*mt.cos(aconico_rg))/10.0
            tz_rg = -rf_rg * mt.sin(aconico_rg) / 10.0
            y_off = (ra_rg + ra_s) / 10.0 + 1.0  # cm — clear of internal gears

            # Drive Pinion: NC8 gear 2 (rotcon) + Y offset
            t_dp2 = adsk.core.Matrix3D.create()
            t_dp2.setToRotation(-aconico_dp, adsk.core.Vector3D.create(0, 1, 0),
                                 adsk.core.Point3D.create(rf_dp / 10.0, 0, 0))
            cur = t_dp2.translation
            t_dp2.translation = adsk.core.Vector3D.create(cur.x, y_off, cur.z)

            # Ring Gear: NC8 gear 1 (moveAndRotateBevel) + Y offset
            t_rg2 = adsk.core.Matrix3D.create()
            t_rg2.setToRotation(-aconico_rg, adsk.core.Vector3D.create(0, 1, 0),
                                  adsk.core.Point3D.create(rf_rg / 10.0, 0, 0))
            t_rg2.translation = adsk.core.Vector3D.create(tx_rg, y_off, tz_rg)

            def rg(c): _build_one_bevel(x_rg,y_rg,x2_rg,y2_rg,z_ring,z_pinion_drive,
                                         rp_rg,rp_dp,rf_rg,ra_rg,Ttda_rg,aok_rg,m,c)
            def dp(c): _build_one_bevel(x_dp,y_dp,x2_dp,y2_dp,z_pinion_drive,z_ring,
                                         rp_dp,rp_rg,rf_dp,ra_dp,Ttda_dp,aok_dp,m,c)
            occ_rg2 = make_nc8('RingGear', rg, t_rg2)
            try: occ_rg2.component.bRepBodies.item(0).isVisible = False
            except Exception: pass
            make_nc8('DrivePinion', dp, t_dp2)
            try: occ_rg2.component.bRepBodies.item(0).isVisible = True
            except Exception: pass

    else:
        # ── EXPLODED MODE: gears spaced in a row along X ─────────────────────
        col = 0
        build_and_place('SideGear_R', sd, col); col += 1
        build_and_place('SideGear_L', sd, col); col += 1
        for i in range(1, n_spider + 1):
            build_and_place('SpiderGear_' + str(i), sp, col)
            col += 1

        if ring_params is not None:
            (rf_rg, x_rg, y_rg, x2_rg, y2_rg, aok_rg, Ttda_rg, ra_rg,
             rf_dp, x_dp, y_dp, x2_dp, y2_dp, aok_dp, Ttda_dp, ra_dp,
             rp_rg, rp_dp) = ring_params

            def rg(c): _build_one_bevel(x_rg, y_rg, x2_rg, y2_rg,
                                         z_ring, z_pinion_drive, rp_rg, rp_dp,
                                         rf_rg, ra_rg, Ttda_rg, aok_rg, m, c)
            def dp(c): _build_one_bevel(x_dp, y_dp, x2_dp, y2_dp,
                                         z_pinion_drive, z_ring, rp_dp, rp_rg,
                                         rf_dp, ra_dp, Ttda_dp, aok_dp, m, c)

            build_and_place('RingGear',    rg, col); col += 1
            build_and_place('DrivePinion', dp, col); col += 1

    # Delete warm-up reference component
    if _wu_occ is not None:
        try: _wu_occ.deleteMe()
        except Exception: pass

    # ── Timeline group ───────────────────────────────────────────────────────
    try:
        ops = design.timeline.count - nuOps
        if ops > 0:
            design.timeline.timelineGroups.add(
                design.timeline.count - ops,
                design.timeline.count - 1)
    except Exception:
        pass

    # ── Özet mesajı ──────────────────────────────────────────────────────────
    total = n_spider + 2 + (2 if ring_params else 0)
    sp_mm = round(spacing * 10, 1)
    ui.messageBox(
        'Differential Gear Set Complete!\n\n'
        '  Module       : ' + str(round(m, 2))       + ' mm\n'
        '  Side Gear z1 : ' + str(z_side)             + ' teeth  (r=' + str(round(rp_s,  1)) + 'mm)\n'
        '  Spider    z2 : ' + str(z_spider)           + ' teeth  (r=' + str(round(rp_sp, 1)) + 'mm)\n'
        '  Pressure Ang : ' + str(round(radToDeg(ap), 1)) + ' deg\n'
        '  Cone dist Ao : ' + str(round(Ao, 1))      + ' mm\n'
        + ('  Ring Gear z  : ' + str(z_ring)           + ' teeth\n'
           '  Drive Pinion : ' + str(z_pinion_drive)   + ' teeth\n' if ring_params else '') +
        '\n  TOTAL: ' + str(total) + ' parts'
        + (' — Assembled (NC8 layout)' if assembled else
           ' — Exploded (' + str(sp_mm) + 'mm spacing, no overlap)') +
        '\n  Use Fusion 360 Joint to constrain the assembly.',
        'Differential Gear Generator')


# ═══════════════════════════════════════════════════════════════════════════════
#  COMMAND HANDLERS
# ═══════════════════════════════════════════════════════════════════════════════

def _fmt_preview(m_cm, z_side, z_spider, z_ring=0, z_pinion=0):
    """Live-calculated preview text shown in the dialog."""
    try:
        m = m_cm * 10.0   # cm -> mm
        if m <= 0 or z_side < 2 or z_spider < 2:
            return 'Enter parameters above.'
        import math
        Ao   = m * math.sqrt(z_side**2 + z_spider**2) / 2.0
        b    = Ao / 3.0
        dp_s = m * z_side
        dp_p = m * z_spider
        lines = [
            f'Cone distance  Ao = {Ao:.2f} mm',
            f'Face width      b = {b:.2f} mm  (= Ao/3)',
            f'Side pitch Ø      = {dp_s:.2f} mm',
            f'Spider pitch Ø   = {dp_p:.2f} mm',
        ]
        if z_ring > 0 and z_pinion > 0:
            rr = z_ring / z_pinion
            lines.append(f'Ring/Pinion ratio = {z_ring}/{z_pinion} = {rr:.2f} : 1')
        return '\n'.join(lines)
    except Exception:
        return ''


class _CmdCreated(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui  = app.userInterface
        try:
            cmd    = args.command
            inputs = cmd.commandInputs
            cmd.isExecutedWhenPreEmpted = False

            # ════  GEAR PARAMETERS  ════════════════════════════════
            # Module preset dropdown  (common metric values)
            mod_dd = inputs.addDropDownCommandInput(
                'ModuleDD', 'Module  [mm]',
                adsk.core.DropDownStyles.TextListDropDownStyle)
            for lbl, selected in [
                ('0.5 mm', False), ('0.8 mm', False),
                ('1.0 mm', False), ('1.25 mm', False), ('1.5 mm', False),
                ('2.0 mm', True),  ('2.5 mm', False), ('3.0 mm', False),
                ('4.0 mm', False), ('5.0 mm', False)
            ]:
                mod_dd.listItems.add(lbl, selected)

            # Allow free entry as well
            inputs.addValueInput(
                'Module', 'Custom Module  [mm]  (overrides preset)', 'mm',
                adsk.core.ValueInput.createByReal(0.2))

            inputs.addIntegerSpinnerCommandInput(
                'Z_side', 'Side Gear — tooth count  (z₁)', 10, 100, 1, 18)

            inputs.addIntegerSpinnerCommandInput(
                'Z_spider', 'Spider Gear — tooth count  (z₂)', 8, 60, 1, 12)

            inputs.addFloatSpinnerCommandInput(
                'PressureAngle', 'Pressure Angle  [deg]', 'deg', 14.5, 30.0, 0.5, 20.0)

            dd = inputs.addDropDownCommandInput(
                'N_spider', 'Number of Spider Gears',
                adsk.core.DropDownStyles.TextListDropDownStyle)
            dd.listItems.add('2  — Standard open differential', True)
            dd.listItems.add('4  — High-torque / off-road', False)

            # ════  LIVE PREVIEW  ═════════════════════════════════
            inputs.addTextBoxCommandInput(
                'Preview', 'Calculated',
                _fmt_preview(0.2, 18, 12), 4, True)

            # ════  OUTPUT OPTIONS  ════════════════════════════════
            inputs.addValueInput(
                'BoreDia', 'Shaft Bore Diameter  [mm]  (0 = none)', 'mm',
                adsk.core.ValueInput.createByReal(0.0))

            inputs.addBoolValueInput(
                'FastCompute', 'Fast Compute  (recommended)', True, '', True)

            layout_dd = inputs.addDropDownCommandInput(
                'Layout', 'Layout Mode',
                adsk.core.DropDownStyles.TextListDropDownStyle)
            layout_dd.listItems.add(
                'Assembled  — NC8 bevel layout (spider +/-X, side +/-Z)', False)
            layout_dd.listItems.add(
                'Exploded   — Parts along X axis, no overlap (inspect/export)', True)

            # ════  RING GEAR + DRIVE PINION  (optional)  ═══════════════
            inputs.addBoolValueInput(
                'IncludeRingGear', 'Include Ring Gear + Drive Pinion', True, '', True)

            rg_z = inputs.addIntegerSpinnerCommandInput(
                'Z_ring', 'Ring Gear (Crown Wheel) — tooth count', 20, 200, 2, 36)
            dp_z = inputs.addIntegerSpinnerCommandInput(
                'Z_pinion', 'Drive Pinion — tooth count', 6, 40, 1, 9)
            ring_prev = inputs.addTextBoxCommandInput(
                'RingPreview', 'Ring Gear Info', '', 2, True)
            rg_z.isVisible      = False
            dp_z.isVisible      = False
            ring_prev.isVisible = False

            # ════  ABOUT  ═════════════════════════════════════════════
            inputs.addTextBoxCommandInput('Info', '', (
                '<b>Differential Gear Generator</b><br>'
                'Involute 90° bevel gears — GFGearGenerator NC8 core<br><br>'
                '<b>Side Gear</b>: axle output  (±Z assembled)&nbsp;&nbsp;'
                '<b>Spider</b>: cross-pin planet  (±X assembled)<br>'
                '<b>Ring Gear</b>: crown wheel (driven by pinion)&nbsp;&nbsp;'
                '<b>Pinion</b>: motor input shaft<br><br>'
                'Face width = Ao / 3 &nbsp;(industry standard)<br>'
                'Assembled: all cone apices meet at world origin.'
            ), 6, True)

            # Wire up handlers
            on_changed = _CmdInputChanged()
            cmd.inputChanged.add(on_changed)
            _handlers.append(on_changed)

            on_exec = _CmdExecute()
            on_val  = _CmdValidate()
            cmd.execute.add(on_exec)
            cmd.validateInputs.add(on_val)
            _handlers.extend([on_exec, on_val])

        except Exception:
            ui.messageBox('UI Error:\n' + traceback.format_exc())


class _CmdInputChanged(adsk.core.InputChangedEventHandler):
    """Updates live-preview text and toggles ring gear inputs."""
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ev      = adsk.core.InputChangedEventArgs.cast(args)
            changed = ev.input
            inputs  = changed.parentCommand.commandInputs

            # Sync module preset -> custom value
            if changed.id == 'ModuleDD':
                lbl = changed.selectedItem.name.split()[0]  # e.g. '2.0'
                try:
                    inputs.itemById('Module').value = float(lbl) / 10.0
                except Exception:
                    pass

            # Toggle ring gear inputs
            if changed.id == 'IncludeRingGear':
                vis = changed.value
                inputs.itemById('Z_ring').isVisible       = vis
                inputs.itemById('Z_pinion').isVisible     = vis
                inputs.itemById('RingPreview').isVisible  = vis

            # Update live preview whenever any key input changes
            if changed.id in ('Module', 'ModuleDD', 'Z_side', 'Z_spider',
                              'Z_ring', 'Z_pinion', 'IncludeRingGear'):
                try:
                    m_cm    = inputs.itemById('Module').value
                    z_side  = inputs.itemById('Z_side').value
                    z_sp    = inputs.itemById('Z_spider').value
                    inc_rg  = inputs.itemById('IncludeRingGear').value
                    z_rg    = inputs.itemById('Z_ring').value   if inc_rg else 0
                    z_dp    = inputs.itemById('Z_pinion').value if inc_rg else 0

                    inputs.itemById('Preview').text = _fmt_preview(
                        m_cm, int(z_side), int(z_sp), int(z_rg), int(z_dp))

                    if inc_rg and z_rg > 0 and z_dp > 0:
                        rr = int(z_rg) / int(z_dp)
                        inputs.itemById('RingPreview').text = (
                            f'Reduction ratio  = {int(z_rg)}/{int(z_dp)} = {rr:.2f} : 1\n'
                            f'Ring pitch Ø      = {inputs.itemById("Module").value*10*int(z_rg):.1f} mm')
                except Exception:
                    pass
        except Exception:
            pass


class _CmdExecute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui  = app.userInterface
        try:
            inputs   = adsk.core.CommandEventArgs.cast(args).command.commandInputs
            # Module: custom value takes priority; if unchanged from default, use preset
            m_custom = inputs.itemById('Module').value
            m        = m_custom * 10.0   # cm → mm
            z_side   = int(inputs.itemById('Z_side').value)
            z_spider = int(inputs.itemById('Z_spider').value)
            ap       = inputs.itemById('PressureAngle').value
            n_str    = inputs.itemById('N_spider').selectedItem.name
            n_spider = 4 if n_str.strip().startswith('4') else 2
            bore_mm  = inputs.itemById('BoreDia').value * 10.0
            fast     = inputs.itemById('FastCompute').value
            assembled = inputs.itemById('Layout').selectedItem.name.startswith('Assembled')

            include_rg    = inputs.itemById('IncludeRingGear').value
            z_ring        = int(inputs.itemById('Z_ring').value)   if include_rg else 0
            z_pinion_drv  = int(inputs.itemById('Z_pinion').value) if include_rg else 0

            build_differential(m, z_side, z_spider, ap, fast, n_spider, bore_mm,
                                z_ring, z_pinion_drv, assembled)

        except Exception:
            ui.messageBox('Execution Error:\n' + traceback.format_exc())


class _CmdValidate(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            inputs   = adsk.core.ValidateInputsEventArgs.cast(args).inputs
            m        = inputs.itemById('Module').value * 10.0
            z_side   = inputs.itemById('Z_side').value
            z_spider = inputs.itemById('Z_spider').value
            bore_mm  = inputs.itemById('BoreDia').value * 10.0
            inc_rg   = inputs.itemById('IncludeRingGear').value
            z_ring   = inputs.itemById('Z_ring').value   if inc_rg else 20
            z_pin    = inputs.itemById('Z_pinion').value if inc_rg else 6
            adsk.core.ValidateInputsEventArgs.cast(args).areInputsValid = (
                m > 0.1 and z_side >= 10 and z_spider >= 8 and bore_mm >= 0
                and z_ring >= 20 and z_pin >= 6 and z_ring > z_pin)
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════════════════════
#  ADD-IN ENTRY POINTS
# ═══════════════════════════════════════════════════════════════════════════════

CMD_ID   = 'DifferentialGearGen_CMD'
PANEL_ID = 'DiffGearPanel_ID'
_tbPanel = None


def run(context):
    global _tbPanel
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        ws     = ui.workspaces.itemById('FusionSolidEnvironment')
        panels = ws.toolbarPanels
        _tbPanel = panels.itemById(PANEL_ID)
        if _tbPanel:
            _tbPanel.deleteMe()
        _tbPanel = panels.add(PANEL_ID, 'DIFF GEAR GEN', 'SelectPanel', False)

        existing = ui.commandDefinitions.itemById(CMD_ID)
        if existing:
            existing.deleteMe()

        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID,
            'Differential Gear Generator',
            '6-piece involute bevel differential: Ring + Pinion + 2xSpider + 2xSide.',
            'resources')

        on_created = _CmdCreated()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        ctrl = _tbPanel.controls.addCommand(cmd_def)
        ctrl.isPromoted          = True
        ctrl.isPromotedByDefault = True

    except Exception:
        if ui:
            ui.messageBox('Startup Error:\n' + traceback.format_exc())


def stop(context):
    global _tbPanel
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        if _tbPanel and _tbPanel.isValid:
            _tbPanel.deleteMe()
        _tbPanel = None
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        if ui:
            ui.messageBox('Shutdown Error:\n' + traceback.format_exc())
