# (AeroTrace 2026-09-19) Correctif NimBLE-Arduino 2.x — NimBLERemoteValueAttribute::onReadCB copie
# OS_MBUF_PKTLEN(om) octets depuis le PREMIER mbuf seulement (attr->om->om_data). Une valeur lue > ~275 o
# arrive en CHAÎNE d'mbufs (bloc 292 o) → tout ce qui suit le 1er bloc est de la mémoire quelconque.
# Vécu : CHR_FLIGHTS 453 o → JSON corrompu à l'octet 275 → « No flights on SD » sur l'écran (bug toujours
# présent en amont, master 09/2026). Script PlatformIO « pre: » : patch idempotent après téléchargement.
Import("env")
import os
src = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env["PIOENV"], "NimBLE-Arduino", "src", "NimBLERemoteValueAttribute.cpp")
OLD = "valBuf->append(attr->om->om_data, data_len);"
NEW = ("{ /* AeroTrace : copie a travers la CHAINE d'mbufs (bug amont : 1er bloc seulement) */\n"
       "                    uint8_t* flat = (uint8_t*)malloc(data_len ? data_len : 1);\n"
       "                    if (flat) { os_mbuf_copydata(attr->om, 0, data_len, flat); valBuf->append(flat, data_len); free(flat); }\n"
       "                }")
if os.path.exists(src):
    s = open(src, encoding="utf-8").read()
    if OLD in s:
        open(src, "w", encoding="utf-8").write(s.replace(OLD, NEW))
        print("[patch_nimble_readcb] onReadCB patché (copie mbuf chain) :", src)
    elif "AeroTrace : copie a travers" in s:
        print("[patch_nimble_readcb] déjà patché")
    else:
        print("[patch_nimble_readcb] ATTENTION : motif introuvable — version NimBLE différente, vérifier onReadCB")
else:
    print("[patch_nimble_readcb] lib absente (1er build : sera patchée au build suivant)")
