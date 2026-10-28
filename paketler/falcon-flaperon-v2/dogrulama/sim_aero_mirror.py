"""AeroModel.hh aynası + toplamsal v2 flaperon uzantısı (Codex şerit G, 2026-10-28). Yalnız stdout."""
import ast
import bisect
import copy
import math
import re
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "girdi" / "falcon-gazebo-sim" / "config" / "aero_v1_config.yaml"
COEFFS = ("CL", "CD", "Cm", "CY", "Cl", "Cn")


def _numeric_yaml(text):
    """PyYAML yoksa: sayisal blok eslemeleri, akis/blok sayi listeleri.
    Metinsel aciklama/ustveriler hesapta kullanilmaz; YAML alias/tag desteklenmez.
    """
    root, stack = {}, []
    lines = iter(text.splitlines())
    for raw in lines:
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        while stack and stack[-1][0] >= indent:
            stack.pop()
        m = re.match(r"^\s*([A-Za-z_][\w]*):\s*(.*)$", line)
        if m:
            key, value = m.groups()
            if value.startswith("["):
                while "]" not in value:
                    value += next(lines).split("#", 1)[0].strip()
            parent = root
            for _, k in stack:
                parent = parent[k]
            if not value:
                parent[key] = {}
                stack.append((indent, key))
            else:
                try:
                    x = ast.literal_eval(value) if value.startswith("[") else float(value)
                    if isinstance(x, list):
                        x = [float(v) for v in x]
                    parent[key] = x
                except (ValueError, SyntaxError):
                    pass
        elif line.strip().startswith("- ") and stack:
            try:
                x = float(line.strip()[2:])
            except ValueError:
                continue
            parent = root
            for _, k in stack[:-1]:
                parent = parent[k]
            key = stack[-1][1]
            if isinstance(parent[key], dict):
                parent[key] = []
            parent[key].append(x)
    return root


def load_config(path):
    """Dosya yolu; test/entegrasyon icin ayni semada dict de kabul edilir."""
    if isinstance(path, dict):
        root = copy.deepcopy(path)
    else:
        text = Path(path).read_text(encoding="utf-8-sig")
        try:
            import yaml
        except ImportError:
            root = _numeric_yaml(text)
        else:
            root = yaml.safe_load(text)
    c = {}
    fields = {
        "reference_geometry": {
            "S": "wing_area_S_m2",
            "b": "wingspan_b_m",
            "c_ref": "reference_chord_c_ref_m",
        },
        "environment": {
            "rho": "air_density_rho_kg_m3",
            "vSafeFloor": "v_safe_floor_m_s",
        },
        "longitudinal": {
            "CLa": "CLa_per_rad",
            "Cma": "Cma_per_rad",
            "CLq": "CLq",
            "Cmq": "Cmq",
            "CL0": "CL0",
            "Cm0": "Cm0",
        },
        "lateral_directional": {
            k: k for k in (
                "CYb", "CYp", "CYr", "Clb", "Clp", "Clr",
                "Cnb", "Cnp", "Cnr",
            )
        },
        "drag_polar": {"CD0": "CD0", "dragK": "k"},
        "high_alpha_limiter": {"CLmax": "CLmax_manufacturer"},
    }
    for section, names in fields.items():
        for dst, key in names.items():
            c[dst] = float(root[section][key])
    c["Cldr"] = float(root["lateral_directional"]["Cldr_per_rad"])
    c["alphaTransition"] = math.radians(
        root["high_alpha_limiter"]["alpha_transition_deg"]
    )
    c["mapping"] = dict(root["control_mapping"])
    for key in ("flap_left_down_sign", "flap_right_down_sign"):
        c["mapping"].setdefault(key, -1.0)
    if not all(math.isfinite(x) for x in c.values() if isinstance(x, float)):
        raise ValueError("Sonlu olmayan katsayi")
    if min(c[k] for k in ("S", "b", "c_ref", "rho", "vSafeFloor")) <= 0:
        raise ValueError("Geometri, yogunluk ve hiz tabani pozitif olmali")

    t = c["alphaTransition"]
    c["hpos"] = c["CLmax"] - (c["CL0"] + c["CLa"] * t)
    c["kpos"] = c["CLa"] / c["hpos"] if c["hpos"] > 1e-9 else 0.0
    c["aneg"] = c["CL0"] - c["CLa"] * t + c["CLmax"]
    c["kneg"] = c["CLa"] / c["aneg"] if c["aneg"] > 1e-9 else 0.0

    lookup = root["control_surface_lookup"]
    c["tables"] = {}
    for name in (
        "elevator", "aileron", "rudder",
        "flaperon_symmetric", "flaperon_antisymmetric", "flaperon_side",
    ):
        flap = name.startswith("flaperon")
        if flap and name not in lookup:
            continue
        node = lookup[name]
        deg = list(map(
            float,
            node.get("breakpoints_deg", lookup.get("breakpoints_deg", [])),
        ))
        if (
            len(deg) < 2
            or not all(math.isfinite(x) for x in deg)
            or any(a >= b for a, b in zip(deg, deg[1:]))
            or 0.0 not in deg
        ):
            raise ValueError(name + ": artan, sifir iceren izgara gerekli")
        bound = float(node.get(
            "domain_bound_deg",
            lookup.get("domain_bound_deg", max(abs(x) for x in deg)),
        ))
        if (
            not math.isfinite(bound) or bound <= 0
            or deg[0] > -bound or deg[-1] < bound
        ):
            raise ValueError(name + ": izgara domain_bound_deg araligini kapsamali")
        table = {
            "x": [math.radians(x) for x in deg],
            "bound": math.radians(bound),
        }
        zero = deg.index(0.0)
        for k in COEFFS:
            key = "d" + k
            full = k + "_full"
            if name in ("aileron", "rudder") and full in node:
                a = [float(x) for x in node[full]]
                if len(a) != len(deg):
                    raise ValueError(name + "." + full + ": uzunluk")
                values = [x - a[zero] for x in a]
            elif name == "rudder" and k == "Cl":
                values = [c["Cldr"] * math.radians(x) for x in deg]
            elif key in node:
                values = [float(x) for x in node[key]]
            elif (
                name in ("aileron", "rudder")
                and k in ("CY", "Cl", "Cn") and k in node
            ):
                values = [float(x) for x in node[k]]
            else:
                required = (
                    flap
                    or (name == "elevator" and k in ("CL", "CD", "Cm"))
                    or (name == "aileron" and k in COEFFS)
                    or (name == "rudder" and k in ("CD", "CY", "Cn"))
                )
                if required:
                    raise ValueError(name + ": eksik " + key)
                values = [0.0] * len(deg)
            if len(values) != len(deg) or not all(math.isfinite(x) for x in values):
                raise ValueError(name + "." + key + ": uzunluk/sonluluk")
            if flap and values[zero] != 0.0:
                raise ValueError(name + "." + key + ": sifir sapmada tam 0 gerekli")
            table[k] = values
        c["tables"][name] = table
    return c


def _interp(table, key, x):
    x = max(-table["bound"], min(table["bound"], x))
    grid, values = table["x"], table[key]
    if x <= grid[0]:
        return values[0]
    if x >= grid[-1]:
        return values[-1]
    j = bisect.bisect_right(grid, x)
    f = (x - grid[j-1]) / (grid[j] - grid[j-1])
    return values[j-1] + f * (values[j] - values[j-1])


def saturated_cl(c, alpha):
    t = c["alphaTransition"]
    if alpha > t:
        return c["CLmax"] - c["hpos"] * math.exp(-c["kpos"] * (alpha-t))
    if alpha < -t:
        return -c["CLmax"] + c["aneg"] * math.exp(c["kneg"] * (alpha+t))
    return c["CL0"] + c["CLa"] * alpha


def compute_aero(cfg, state):
    """Giris dict; eksik durum alanlari 0. Acilar rad, FLU kuvvet N/moment Nm."""
    c, s = cfg, state
    u, v, w = (float(s.get(k, 0.0)) for k in ("u", "v", "w"))
    v2 = u*u + v*v + w*w
    V = math.sqrt(v2)
    alpha = math.atan2(-w, u)
    beta = math.atan2(v, math.hypot(u, w))
    vs = max(V, c["vSafeFloor"])
    ph, qh, rh = (
        s.get(k, 0.0)*length/(2.0*vs)
        for k, length in (("p", c["b"]), ("q", c["c_ref"]), ("r", c["b"]))
    )
    contributions = []
    for name, delta in (
        ("elevator", "deltaE"),
        ("aileron", "deltaA"),
        ("rudder", "deltaR"),
        ("flaperon_symmetric", "deltaFSym"),
        ("flaperon_antisymmetric", "deltaFAnti"),
    ):
        table = c["tables"].get(name)
        if table is not None:
            contributions.append({
                k: _interp(table, k, s.get(delta, 0.0)) for k in COEFFS
            })
        elif s.get(delta, 0.0) != 0.0 and not (
            name.startswith("flaperon") and "flaperon_side" in c["tables"]
        ):
            raise ValueError(name + ": sifir disi komut icin tablo gerekli")
    # v2 yan tablosu: sol flap kendi fiziksel sapmasiyla (firar kenari asagi +); sag ayna
    # (CY, Cl, Cn isareti ters). deltaFL/deltaFR verilmemisse sim./antisim.'den turetilir.
    side = c["tables"].get("flaperon_side")
    if side is not None:
        fs, fa = s.get("deltaFSym", 0.0), s.get("deltaFAnti", 0.0)
        fl = s.get("deltaFL", fs + fa)
        fr = s.get("deltaFR", fs - fa)
        for d, mirror in ((fl, 1.0), (fr, -1.0)):
            contributions.append({
                k: _interp(side, k, d) * (mirror if k in ("CY", "Cl", "Cn") else 1.0)
                for k in COEFFS
            })
    add = {k: sum(a[k] for a in contributions) for k in COEFFS}

    cm_static = c["Cm0"] + c["Cma"]*alpha + add["Cm"]
    cm_rate = c["Cmq"]*qh
    CL = saturated_cl(c, alpha) + c["CLq"]*qh + add["CL"]
    CD = max(c["CD0"] + c["dragK"]*CL*CL + add["CD"], c["CD0"])
    CY = c["CYb"]*beta + c["CYp"]*ph + c["CYr"]*rh + add["CY"]
    Cl = c["Clb"]*beta + c["Clp"]*ph + c["Clr"]*rh + add["Cl"]
    Cn = c["Cnb"]*beta + c["Cnp"]*ph + c["Cnr"]*rh + add["Cn"]

    qbar = 0.5*c["rho"]*v2
    L, D, Y = (qbar*c["S"]*x for x in (CL, CD, CY))
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    force = (
        -D*ca*cb - Y*ca*sb + L*sa,
        -D*sb + Y*cb,
        D*sa*cb + Y*sa*sb + L*ca,
    )
    moment = (
        qbar*c["S"]*c["b"]*Cl,
        qbar*c["S"]*c["c_ref"]*(-cm_static + cm_rate),
        qbar*c["S"]*c["b"]*Cn,
    )
    return dict(
        V=V, alpha=alpha, beta=beta, qbar=qbar,
        CL=CL, CD=CD, CY=CY, Cl=Cl, Cm=cm_static+cm_rate, Cn=Cn,
        cmStatic=cm_static, cmRate=cm_rate,
        forceBody=force, momentBody=moment,
        L_N=L, D_N=D, pHat=ph, qHat=qh, rHat=rh,
    )


def joint_to_deltas(theta, cfg):
    """Anahtarlar SDF joint adlari; eksik eklem eklentideki gibi 0 okunur."""
    m = cfg["mapping"]
    get = lambda name: float(theta.get(name + "_joint", 0.0))
    lf = m["flap_left_down_sign"]*get("left_flap")
    rf = m["flap_right_down_sign"]*get("right_flap")
    return dict(
        deltaA=0.5*m["aileron_sign"]*(get("right_aileron")-get("left_aileron")),
        deltaE=0.5*m["elevator_sign"]*(get("left_elevator")+get("right_elevator")),
        deltaR=m["rudder_sign"]*get("rudder"),
        deltaFSym=0.5*(lf+rf),
        deltaFAnti=0.5*(lf-rf),
        deltaFL=lf,
        deltaFR=rf,
    )


def _bracketed_newton(f, lo, hi, tol=1e-12):
    """Sayisal turevli Newton; aralik disina cikarsa ikiye bol."""
    flo, fhi = f(lo), f(hi)
    if flo == 0:
        return lo
    if fhi == 0:
        return hi
    if flo*fhi > 0:
        raise ValueError("Kok kusatilmadi")
    x = (lo+hi)*0.5
    for _ in range(120):
        fx = f(x)
        if abs(fx) < tol:
            return x
        if flo*fx <= 0:
            hi, fhi = x, fx
        else:
            lo, flo = x, fx
        if hi-lo < 1e-13:
            return (lo+hi)*0.5
        h = min(1e-6, (hi-lo)*0.1)
        slope = (f(x+h)-f(x-h))/(2*h)
        candidate = x-fx/slope if slope != 0 else math.inf
        x = candidate if lo < candidate < hi else (lo+hi)*0.5
    raise RuntimeError("Newton yakinsamadi")


def trim(cfg, V, mass, flap_sym_rad=0):
    """beta=p=q=r=gamma=0; ruzgar ekseninde L=mg, Cm_static=0, T=D.
    Cm dogrusal alfa terimiyle elenir; kalan denklem kusatilmali Newton ile.
    Birden fazla kok varsa en kucuk |alfa| secilir. g=9.80665 m/s^2.
    """
    if V <= 0 or mass <= 0 or cfg["Cma"] == 0:
        raise ValueError("V,mass pozitif ve Cma sifirdan farkli olmali")
    table = cfg["tables"]["elevator"]
    bound = table["bound"]
    target = mass*9.80665/(0.5*cfg["rho"]*V*V*cfg["S"])

    def evaluate(de):
        base = dict(u=V, deltaE=de, deltaFSym=flap_sym_rad)
        at0 = compute_aero(cfg, base)
        alpha = -at0["cmStatic"]/cfg["Cma"]
        base.update(u=V*math.cos(alpha), w=-V*math.sin(alpha))
        return alpha, compute_aero(cfg, base)

    def residual(de):
        return evaluate(de)[1]["CL"]-target

    # Izgara kiriklarini ve aralarindaki 32 alt araligi tara.
    knots = sorted(set(
        [-bound, bound] + [x for x in table["x"] if -bound < x < bound]
    ))
    grid = [
        a+(b-a)*i/32
        for a, b in zip(knots, knots[1:])
        for i in range(32)
    ] + [bound]
    roots = []
    for lo, hi in zip(grid, grid[1:]):
        if residual(lo)*residual(hi) <= 0:
            de = _bracketed_newton(residual, lo, hi)
            alpha, out = evaluate(de)
            if abs(alpha) < math.pi/2 and abs(out["cmStatic"]) < 1e-9:
                roots.append((abs(alpha), alpha, de, out))
    if not roots:
        raise ValueError(f"{V:g} m/s: tablo sinirlari icinde yatay trim yok")
    _, alpha, de, out = min(roots, key=lambda x: x[0])
    if abs(out["L_N"]-mass*9.80665) > 1e-7:
        raise RuntimeError("Trim kuvvet kalintisi")
    return dict(
        alpha_deg=math.degrees(alpha),
        deltaE_deg=math.degrees(de),
        CL=out["CL"], CD=out["CD"],
        **{"L/D": out["CL"]/out["CD"]},
        T_N=out["D_N"],
    )


def steady_roll(
    cfg, V, deltaA_rad, deltaR_rad=0, flap_sym_rad=0, flap_anti_rad=0
):
    """Yalniz Cl dengesi: beta=0,r=0; sapma/yan kuvvet dengesi cozulmez.
    pHat=p*b/(2V), boyutsuz yuvarlanma hizidir.
    """
    if V <= 0 or cfg["Clp"] == 0:
        raise ValueError("V pozitif, Clp sifirdan farkli olmali")
    state = dict(
        u=V, deltaA=deltaA_rad, deltaR=deltaR_rad,
        deltaFSym=flap_sym_rad, deltaFAnti=flap_anti_rad,
    )
    out = compute_aero(cfg, state)
    ph = -out["Cl"]/cfg["Clp"]
    p = ph*2*max(V, cfg["vSafeFloor"])/cfg["b"]
    return dict(pHat=ph, p_rad_s=p, p_deg_s=math.degrees(p))


def self_test(path=DEFAULT_PATH):
    c = load_config(path)
    z = compute_aero(c, {})
    for key, expected in (
        ("CL", 0.437035),
        ("Cm", 0.010550),
        ("CD", 0.04518477841668),
    ):
        assert math.isclose(
            z[key], expected, rel_tol=0, abs_tol=1e-13
        ), (key, z[key])
    assert z["forceBody"] == (0.0, 0.0, 0.0)
    assert z["momentBody"] == (0.0, 0.0, 0.0)
    assert z["CY"] == -0.00004 and z["Cn"] == 0.00002

    rate = compute_aero(c, dict(u=18, q=0.2))
    assert math.isclose(
        rate["momentBody"][1],
        rate["qbar"]*c["S"]*c["c_ref"]*(-rate["cmStatic"]+rate["cmRate"]),
    )

    # v2: ortak izgara yok; her yuzeyde sifir indeksi artik 1.
    raw = _numeric_yaml(Path(path).read_text(encoding="utf-8-sig"))
    lookup = raw["control_surface_lookup"]
    old_deg = lookup.pop("breakpoints_deg")
    lookup.pop("domain_bound_deg")
    for name, indices in (
        ("elevator", (3, 7, 11)),
        ("aileron", (2, 7, 12)),
        ("rudder", (1, 7, 13)),
    ):
        node = lookup[name]
        for key, values in list(node.items()):
            if isinstance(values, list):
                node[key] = [values[i] for i in indices]
        node["breakpoints_deg"] = [old_deg[i] for i in indices]
        node["domain_bound_deg"] = abs(node["breakpoints_deg"][0])
    for name, scale in (
        ("flaperon_symmetric", 1.0),
        ("flaperon_antisymmetric", 0.5),
    ):
        lookup[name] = dict(
            breakpoints_deg=[-26.5, 0, 26.5], domain_bound_deg=26.5
        )
        for i, k in enumerate(COEFFS, 1):
            lookup[name]["d"+k] = [-0.01*i*scale, 0, 0.01*i*scale]
    v2 = load_config(raw)
    for name in ("aileron", "rudder"):
        assert _interp(v2["tables"][name], "CD", 0) == 0
    assert compute_aero(v2, {})["CL"] == z["CL"]

    fs = math.radians(26.5)
    fl = compute_aero(v2, dict(u=18, deltaFSym=fs, deltaFAnti=fs))
    assert math.isclose(fl["CL"], z["CL"]+0.015, abs_tol=1e-14)
    assert math.isclose(fl["cmStatic"], z["cmStatic"]+0.045, abs_tol=1e-14)
    assert math.isclose(
        fl["CD"], v2["CD0"]+v2["dragK"]*fl["CL"]**2+0.030
    )
    for k, increment in (("CY", 0.060), ("Cl", 0.075), ("Cn", 0.090)):
        assert math.isclose(fl[k], z[k]+increment, abs_tol=1e-14)
    clipped = compute_aero(
        v2, dict(u=18, deltaFSym=10*fs, deltaFAnti=10*fs)
    )
    for k in COEFFS:
        assert clipped[k] == fl[k]
    low_drag = compute_aero(v2, dict(u=18, deltaFSym=-fs, deltaFAnti=-fs))
    assert low_drag["CD"] == v2["CD0"]

    mapped = joint_to_deltas(dict(
        left_flap_joint=-fs, right_flap_joint=-fs,
        left_aileron_joint=-0.1, right_aileron_joint=0.1,
        left_elevator_joint=0.2, right_elevator_joint=0.2,
        rudder_joint=0.3,
    ), v2)
    assert {k: mapped[k] for k in ("deltaA", "deltaE", "deltaR", "deltaFSym", "deltaFAnti")} == dict(
        deltaA=0.1, deltaE=-0.2, deltaR=0.3,
        deltaFSym=fs, deltaFAnti=0,
    )
    assert mapped["deltaFL"] == fs and mapped["deltaFR"] == fs

    rr = steady_roll(c, 18, math.radians(10))
    balanced = compute_aero(c, dict(
        u=18, deltaA=math.radians(10), p=rr["p_rad_s"]
    ))
    assert abs(balanced["Cl"]) < 1e-14
    t = c["alphaTransition"]
    assert math.isclose(saturated_cl(c, t), c["CL0"]+c["CLa"]*t)
    assert saturated_cl(c, t+10) <= c["CLmax"]
    assert saturated_cl(c, -t-10) >= -c["CLmax"]

    print("v2 PASS: independent grids, zero index=1, flap sums/clamp, joint mapping")
    print("roll/saturation PASS")
    print(
        f"self_test PASS: CL={z['CL']:.6f} "
        f"Cm={z['Cm']:.6f} CD={z['CD']:.14f}"
    )
    print("mass=6 kg; rho=1.225 kg/m3; g=9.80665 m/s2; flap=0")
    print("V_m_s alpha_deg deltaE_deg CL CD L/D T_N")
    for V in (18.162, 13, 15, 18, 22):
        out = trim(c, V, 6.0)
        print(f"{V:.3f} " + " ".join(
            f"{out[k]:.9f}" for k in
            ("alpha_deg", "deltaE_deg", "CL", "CD", "L/D", "T_N")
        ))
    return c


if __name__ == "__main__":
    self_test()
