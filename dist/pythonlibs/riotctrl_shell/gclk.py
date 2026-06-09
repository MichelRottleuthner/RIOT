# Copyright (C) 2022 HAW Hamburg
#
# This file is subject to the terms and conditions of the GNU Lesser
# General Public License v2.1. See the file LICENSE in the top level
# directory for more details.

"""
gclk-related shell interactions

Defines gclk-related shell command interactions
"""

import re

from riotctrl.shell import ShellInteraction, ShellInteractionParser


# ==== Parsers ====
class GCLKDvfsWsaGetParser(ShellInteractionParser):
    def __init__(self):
        self.ws_re = re.compile(r"current flash waitstates: (?P<ws>\d)$")

    def parse(self, cmd_output):
        """
        Parses output of GCLK::dvfs_wsa_get()

        >>> parser = GCLKDvfsWsaGetParser()
        >>> res = parser.parse("current flash waitstates: 4\\n")
        >>> print(res)
        4
        """
        m = self.ws_re.match(cmd_output)
        if m is not None:
            return m.groupdict()["ws"]


class GCLKDvfsDfsInfoParser(ShellInteractionParser):
    CORE_HANDLE_RE = r"core clock handle: (?P<clockhandle>.*)\n"
    ACTIVE_SCALE_SETTING_RE = (
        r"Currently active scale setting: \((?P<activesetting>\d)\)\n"
    )
    SETTING_HEADER_LINE_RE = r"All available scale settings:\n"
    SETTING_LINE_SEP_RE = r"---.*\n"
    TOPO_SCALE_SETTING_LINE_RE = (
        r"\((?P<sid>\d)\) \[(?P<clock>.*)\]@topology\[(?P<tid>\d)\] "
        r"(?P<mode>SCALE_DIRECT|SCALE_UPTREE_RELATIVE|SCALE_INTERMEDIATE_TOPO_AUTO)"
        r"( via (?P<scalepoint>.*) (?P<scaleop>\*|/) (?P<scalevals>.*))?\n"
    )
    TOPO_LINE_RE = r"^\(\d\) .*\n"
    ALLOWED_CORE_SOURCE_RE = r"allowed core source: .*\n"
    # SETTING_LINE_SEP_RE = r"[-]*\n"

    def __init__(self):
        self.dfs_core_handle_re = re.compile(self.CORE_HANDLE_RE)
        self.dfs_active_scale_setting_re = re.compile(self.ACTIVE_SCALE_SETTING_RE)
        self.dfs_setting_line_sep_re = re.compile(self.SETTING_LINE_SEP_RE)
        self.dfs_topo_line_re = re.compile(self.TOPO_LINE_RE)
        self.dfs_topo_scale_setting_line_re = re.compile(
            self.TOPO_SCALE_SETTING_LINE_RE
        )
        self.dfs_allowed_src_line_re = re.compile(self.ALLOWED_CORE_SOURCE_RE)
        self.dfs_info_re = re.compile(
            self.CORE_HANDLE_RE
            + self.ACTIVE_SCALE_SETTING_RE
            + self.SETTING_HEADER_LINE_RE
            + self.SETTING_LINE_SEP_RE
            + "(?P<settinglines>("
            + self.TOPO_LINE_RE
            + ")*)"
            + self.SETTING_LINE_SEP_RE
            + self.ALLOWED_CORE_SOURCE_RE
            + self.ALLOWED_CORE_SOURCE_RE,
            re.MULTILINE,
        )

    def parse(self, cmd_output):
        """
        Parses output of GCLK::dvfs_dfs_info()

        >>> parser = GCLKDvfsDfsInfoParser()
        >>> res = parser.parse(
        ... "core clock handle: SYSCLK\\n"
        ... "Currently active scale setting: (1)\\n"
        ... "All available scale settings:\\n"
        ... "------------------------------------------\\n"
        ... "(0) [SYSCLK]@topology[1] SCALE_DIRECT via MSIRANGE * 1 2 4 8 10 20 40 80 160 240 320 480\\n"
        ... "(1) [SYSCLK]@topology[6] SCALE_UPTREE_RELATIVE via MSIRANGE * 1 2 4 8 10 20 40 80 160 240 320 480\\n"
        ... "(2) [SYSCLK]@topology[6] SCALE_INTERMEDIATE_TOPO_AUTO\\n"
        ... "(3) [SYSCLK]@topology[7] SCALE_INTERMEDIATE_TOPO_AUTO\\n"
        ... "------------------------------------------\\n"
        ... "allowed core source: HSI16\\n"
        ... "allowed core source: MSI_BASE\\n")
        >>> print(res['clockhandle'])
        SYSCLK
        >>> print(res['activesetting'])
        1
        >>> print([res['scalevariants'][x]['tid'] for x in res['scalevariants']])
        ['1', '6', '6', '7']
        >>> print([res['scalevariants'][x]['mode'] for x in res['scalevariants']])
        ['SCALE_DIRECT', 'SCALE_UPTREE_RELATIVE', 'SCALE_INTERMEDIATE_TOPO_AUTO', 'SCALE_INTERMEDIATE_TOPO_AUTO']
        >>> print([res['scalevariants'][x]['scaleop'] for x in res['scalevariants']])
        ['*', '*', None, None]
        >>> print([res['scalevariants'][x]['scalevals'] for x in res['scalevariants']])
        [[1, 2, 4, 8, 10, 20, 40, 80, 160, 240, 320, 480], [1, 2, 4, 8, 10, 20, 40, 80, 160, 240, 320, 480], None, None]
        """  # noqa: E501

        m = self.dfs_info_re.match(cmd_output)
        if m:
            res = m.groupdict()
            settinglines = res.pop("settinglines")
            scalevariants = {}
            for sl in settinglines.splitlines(True):
                m = self.dfs_topo_scale_setting_line_re.match(sl)
                slprops = m.groupdict()
                if slprops["scalevals"]:
                    # pythonize list of values
                    slprops["scalevals"] = [
                        int(x) for x in slprops["scalevals"].split(" ")
                    ]
                scalevariants[int(slprops.pop("sid"))] = slprops
            res["scalevariants"] = scalevariants
            return res


class GCLKDvfsDvsInfoParser(ShellInteractionParser):
    DVS_POLICY_RE = (
        r"current DVS policy: "
        r"(?P<policy>DVS_PREFER_LOW_VOLTAGE|DVS_PREFER_FAST_FLASH)\n"
    )
    CORE_VOLTAGE_RE = r"current core voltage: (?P<voltage_mv>\d+) mV\n"
    CORE_VOLTAGE_OPTION_LINE_RE = r"core voltage option.*\n"
    CORE_VOLTAGE_OPTION_PARAMS_RE = (
        r"core voltage option (?P<id>\d+): (?P<voltage_mv>\d+) mV\n"
    )

    def __init__(self):
        self.dvs_info_voltage_option_params_re = re.compile(
            self.CORE_VOLTAGE_OPTION_PARAMS_RE
        )
        self.dvs_info_re = re.compile(
            self.DVS_POLICY_RE
            + self.CORE_VOLTAGE_RE
            + "(?P<voltage_options>("
            + self.CORE_VOLTAGE_OPTION_LINE_RE
            + ")*)",
            re.MULTILINE,
        )

    def parse(self, cmd_output):
        """
        Parses output of GCLK::dvfs_dvs_info()

        >>> parser = GCLKDvfsDvsInfoParser()
        >>> res = parser.parse(
        ... "current DVS policy: DVS_PREFER_LOW_VOLTAGE\\n"
        ... "current core voltage: 1200 mV\\n"
        ... "core voltage option 0: 1000 mV\\n"
        ... "core voltage option 1: 1200 mV\\n")
        >>> print(res['policy'])
        DVS_PREFER_LOW_VOLTAGE
        >>> print(res['voltage_mv'])
        1200
        >>> print(res['voltages'])
        {'0': {'voltage_mv': '1000'}, '1': {'voltage_mv': '1200'}}
        """  # noqa: E501

        m = self.dvs_info_re.match(cmd_output)
        if m:
            res = m.groupdict()
            optionlines = res.pop("voltage_options")
            voltages = {}
            for ol in optionlines.splitlines(True):
                m = self.dvs_info_voltage_option_params_re.match(ol)
                olprops = m.groupdict()
                voltages[olprops["id"]] = {"voltage_mv": olprops["voltage_mv"]}

            res["voltages"] = voltages
            return res
        else:
            print("NOMATCH")


class GCLKListParser(ShellInteractionParser):
    def __init__(self):
        self.name_re = re.compile(r"\[(.*)\]$")

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_list()

        >>> parser = GCLKListParser()
        >>> res = parser.parse(
        ... "[SYSCLK]\\n"
        ... "[MSIMUX]\\n"
        ... "[CLK48]\\n"
        ... "[LSCO]\\n"
        ... "[RTC/LTC]\\n"
        ... "[PLL_PREDIV_MUX]\\n"
        ... "[SAI1]\\n")
        >>> sorted(res)
        ['CLK48', 'LSCO', 'MSIMUX', 'PLL_PREDIV_MUX', 'RTC/LTC', 'SAI1', 'SYSCLK']
        >>> len(res)
        7
        """
        res = []
        for line in cmd_output.splitlines():
            m = self.name_re.match(line)
            if m is not None:
                res.append(m.group(1))
                continue
        return res


class GCLKDescribeParser(ShellInteractionParser):
    FRAME_LINE_RE = r"\+-*\+\r\n"
    FREQ_FMT = r"\d+ [k,M]?Hz"
    CLOCK_DESC = (
        r"\[(?P<name>.*)\] +(?P<enable>ON|ON >OFF<|>ON< OFF) +@(?P<freq>"
        + FREQ_FMT
        + r") {(?P<minfreq>"
        + FREQ_FMT
        + r") \.\. (?P<maxfreq>"
        + FREQ_FMT
        + r")}( *|  (?P<scaleop>[/,x])(?P<factor>\d+) {(?P<factors>.*)})\r\n"
    )
    PARENT_LINE_RE = r"  \|----(?P<selected>[>, ])" + CLOCK_DESC
    CLOCK_LINE = r"(?P<clock>.*\r\n)"

    def __init__(self):
        self.box_re = re.compile(
            self.FRAME_LINE_RE
            + r"(("
            + self.PARENT_LINE_RE
            + r")+)"  # match at least one parent line
            r"  \|\r\n"
            r"  \|\r\n" + self.CLOCK_LINE + self.FRAME_LINE_RE
        )

        self.clock_re = re.compile(r"  " + self.CLOCK_DESC)

        self.parent_re = re.compile(self.PARENT_LINE_RE)

    def _normalize_freq_str(self, fstr):
        # may be '1234 MHz' , '123 kHz', etc.
        freq, unit = fstr.split(" ")
        return int(freq) * {"MHz": 1000000, "kHz": 1000, "Hz": 1}[unit]

    # replace parsed values with more pythonic types where appropriate
    def _pythonize_clock_props(self, p):
        name = p.pop("name")
        if "selected" in p:
            p["selected"] = p["selected"] == ">"
        p["gateable"] = p["enable"] in [">ON< OFF", "ON >OFF<"]
        p["enabled"] = p["enable"] in ["ON", ">ON< OFF"]
        p.pop("enable")
        p["factors"] = (
            [f.strip() for f in p["factors"].split(",")] if p["factors"] else None
        )
        if p["factors"]:
            if "..." in p["factors"]:
                p["factors"] = range(int(p["factors"][0]), int(p["factors"][-1]), 1)
            else:
                p["factors"] = [int(x) for x in p["factors"]]

        p.update(
            {k: self._normalize_freq_str(v) for k, v in p.items() if k.endswith("freq")}
        )
        return name, p

    def _parse_clock_props(self, clock_desc_line):
        m = self.clock_re.match(clock_desc_line)
        p = m.groupdict()
        return self._pythonize_clock_props(p)

    def _parse_parent_clock_props(self, clock_desc_line):
        m = self.parent_re.match(clock_desc_line)
        p = m.groupdict()
        return self._pythonize_clock_props(p)

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_describe()

        >>> parser = GCLKDescribeParser()
        >>> res = parser.parse(
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... "  |---->[PLL_M]  ON       @8 MHz {12500 Hz .. 48 MHz}  /6 {1, ..., 8}\\r\\n"
        ... "  |\\r\\n"
        ... "  |\\r\\n"
        ... "  [PLLSAI2_VCO]  ON >OFF< @128 MHz {100 kHz .. 4128 MHz}  x16 {8, ..., 86}\\r\\n"
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... "  |---->[APB2]  ON       @80 MHz {1 Hz .. 2064 MHz}  /1 {1, 2, 4, 8, 16}\\r\\n"
        ... "  |\\r\\n"
        ... "  |\\r\\n"
        ... "  [APB2{x1|x2}]  ON       @80 MHz {1 Hz .. 1980516352 Hz}  x1 {1, 2, 2, 2, 2}\\r\\n"
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... "  |---- [MSISRANGE]  ON       @4 MHz {1 MHz .. 8 MHz}  x40 {10, 20, 40, 80}\\r\\n"
        ... "  |---->[MSIRANGE]   ON       @48 MHz {100 kHz .. 48 MHz}  x480 {1, 2, 4, 8, 10, 20, 40, 80, 160, 240, 320, 480}\\r\\n"
        ... "  |\\r\\n"
        ... "  |\\r\\n"
        ... "  [MSIMUX]  ON       @48 MHz {100 kHz .. 48 MHz} \\r\\n"
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... "  |---->[PLL_VCO] >ON< OFF  @160 MHz {100 kHz .. 4128 MHz}  x20 {8, ..., 86}\\r\\n"
        ... "  |\\r\\n"
        ... "  |\\r\\n"
        ... "  [PLL_Q]  ON >OFF< @80 MHz {12500 Hz .. 2064 MHz}  /2 {2, 4, 6, 8}\\r\\n"
        ... "+----------------------------------------------------------------------------------------------------+\\r\\n"
        ... )
        >>> print([cn for cn, props in res.items()])
        ['PLLSAI2_VCO', 'APB2{x1|x2}', 'MSIMUX', 'PLL_Q']
        >>> print([[p for p in res[cn]['parents']] for cn, props in res.items()])
        [['PLL_M'], ['APB2'], ['MSISRANGE', 'MSIRANGE'], ['PLL_VCO']]
        """  # noqa: E501

        clk_descs = {}
        for match in self.box_re.finditer(cmd_output):
            parent_lines = match.group(1)
            parents = {}
            for pl in parent_lines.splitlines(True):
                name, props = self._parse_parent_clock_props(pl)
                parents[name] = props

            name, props = self._parse_clock_props(match["clock"])
            props["parents"] = parents
            # clk_descs.append({"clock": clock, "parents": parents})
            clk_descs[name] = props

        return clk_descs


class GCLKRootsParser(ShellInteractionParser):
    def __init__(self):
        self.name_re = re.compile(r"\[(.*)\]$")

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_roots()

        >>> parser = GCLKRootsParser()
        >>> res = parser.parse("All possible roots to drive [SYSCLK]\\n"
        ... "[MSI_BASE]\\n"
        ... "[HSI16]\\n"
        ... "[HSE]\\n"
        ... "[NULL]\\n")
        >>> sorted(res)
        ['HSE', 'HSI16', 'MSI_BASE', 'NULL']
        >>> len(res)
        4
        """
        res = []
        for line in cmd_output.splitlines():
            m = self.name_re.match(line)
            if m is not None:
                res.append(m.group(1))
                continue
        return res


class GCLKChildrenParser(ShellInteractionParser):
    def __init__(self):
        self.name_enable_state_re = re.compile(r"\[(.*)\] \((ON|OFF)\)$")

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_children()

        >>> parser = GCLKChildrenParser()
        >>> res = parser.parse("[APB1] (ON)\\n"
        ... "[APB2] (OFF)\\n"
        ... "[AHB/8] (ON)\\n")
        >>> for r in res:
        ...     print(r)
        {'name': 'APB1', 'enabled': True}
        {'name': 'APB2', 'enabled': False}
        {'name': 'AHB/8', 'enabled': True}
        """
        res = []
        for line in cmd_output.splitlines():
            m = self.name_enable_state_re.match(line)
            if m is not None:
                res.append(
                    {
                        "name": m.group(1),
                        "enabled": True if m.group(2) == "ON" else False,
                    }
                )
                continue
        return res


class GCLKAffectedParser(ShellInteractionParser):
    def __init__(self):
        self.affected_state_re = re.compile(
            r"\[(.*)\] (is|is Not) affected .* \[(.*)\].*$"
        )

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_affected()

        >>> parser = GCLKAffectedParser()
        >>> res = parser.parse("[SYSCLK] is Not affected by a change of [MSI]!\\n")
        >>> print(res)
        False
        >>> res = parser.parse("[APB1] is affected by a change of [SYSCLK]!\\n")
        >>> print(res)
        True
        """
        m = self.affected_state_re.match(cmd_output)
        return m.group(2) == "is"


class GCLKFactorParser(ShellInteractionParser):
    def __init__(self):
        self.factor_re = re.compile(r".* (\d*)$")

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_conf_get_factor()

        >>> parser = GCLKFactorParser()
        >>> res = parser.parse("current scale factor of [MSIRANGE] is 480\\n")
        >>> print(res)
        480
        """
        m = self.factor_re.match(cmd_output)
        return m.group(1)


class GCLKFreqParser(ShellInteractionParser):
    def __init__(self):
        self.freq_re = re.compile(r".* is (\d*) Hz$")

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_conf_get_parent()

        >>> parser = GCLKFreqParser()
        >>> res = parser.parse("current scale frequency of [SYSCLK] is 80000000 Hz\\n")
        >>> print(res)
        80000000
        """
        m = self.freq_re.match(cmd_output)
        return m.group(1)


class GCLKParentParser(ShellInteractionParser):
    def __init__(self):
        self.parent_re = re.compile(r"current parent of \[(.*)\] is \[(.*)\]$")

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_conf_get_parent()

        >>> parser = GCLKParentParser()
        >>> res = parser.parse("current parent of [SYSCLK] is [PLL_R]\\n")
        >>> print(res)
        PLL_R
        """
        m = self.parent_re.match(cmd_output)
        return m.group(2)


class GCLKManagerClosestParser(ShellInteractionParser):
    def __init__(self):
        self.result_metadata_re = re.compile(
            r"clockman closest .+ (?P<target_freq>\d+) \d+ (?P<cmpfun>.+) "
            r"((?P<tcidx>\d+)|(?P<listmode>.+))\s+"
            r"max topology for driving .+ employs (?P<maxlen>\d+) clock nodes\s+"
            r"There are (?P<valid_confcnt>\d+) valid configs \(out of (?P<freqcnt>\d+) "
            r"possible configs\) for \[(?P<clkname>.+)\]@(?P<freq>\d+) Hz via "
            r"topology \[(?P<topid>\d+)\]:\s+"
        )

        self.clock_conf_section_re = re.compile(
            (
                r"--------------------\s+"
                r"Valid config IDX (?P<confidx>\d*)\s+"
                r"(?P<rawConfLine>\[.*@.*\](-->\[.*@.*])*)\s+"
                r"LVPOL\| WS: (?P<lvws>\d*) .+: (?P<lvvc>\d*) \((?P<lvmv>\d*) mV\)\s+"
                r"FFPOL\| WS: (?P<ffws>\d*) .+: (?P<ffvc>\d*) \((?P<ffmv>\d*) mV\)\s+"
                r"--------------------\s+"
            )
        )
        self.clock_conf_block_re = re.compile(
            r"\[(?P<clkname>.*)@(?P<freq>\d+)\|((?P<scaleop>/|\*)(?P<factor>\d*))?-\]"
        )
        self.clock_conf_line_re = re.compile(r"\[.*@.*\](-->\[.*@.*])*$")

    @staticmethod
    def _str_elems_to_int(d):
        rd = {}
        for k in d:
            if isinstance(d[k], str):
                try:
                    rd[k] = int(d[k])
                except ValueError:
                    rd[k] = d[k]
            else:
                rd[k] = d[k]
        return rd

    def parse(self, cmd_output):
        """
        Parses output of GCLK::clock_conf_get_parent()

        >>> parser = GCLKManagerClosestParser()
        >>> res = parser.parse(
        ... "clockman closest SYSCLK 8000000 6 exact_leaf listall\\n"
        ... "max topology for driving SYSCLK employs 9 clock nodes\\n"
        ... "There are 4 valid configs (out of 30336 possible configs) for [SYSCLK]@8000000 Hz via topology [6]:\\n"
        ... "all valid configs:\\n"
        ... "--------------------\\n"
        ... "Valid config IDX 0\\n"
        ... "[SYSCLK@8000000|-]-->[PLL_R@8000000|/8-]-->[PLL_VCO@64000000|*16-]-->[PLL_M@4000000|/1-]-->[PLL_PREDIV_MUX@4000000|-]-->[MSI@4000000|-]-->[MSIMUX@4000000|-]-->[MSIRANGE@4000000|*40-]-->[MSI_BASE@100000|-]\\n"
        ... "LVPOL| WS: 1 VCIDX: 0 (1000 mV)\\n"
        ... "FFPOL| WS: 0 VCIDX: 1 (1200 mV)\\n"
        ... "--------------------\\n"
        ... "--------------------\\n"
        ... "Valid config IDX 1\\n"
        ... "[SYSCLK@8000000|-]-->[PLL_R@8000000|/8-]-->[PLL_VCO@64000000|*8-]-->[PLL_M@8000000|/1-]-->[PLL_PREDIV_MUX@8000000|-]-->[MSI@8000000|-]-->[MSIMUX@8000000|-]-->[MSIRANGE@8000000|*80-]-->[MSI_BASE@100000|-]\\n"
        ... "LVPOL| WS: 1 VCIDX: 0 (1000 mV)\\n"
        ... "FFPOL| WS: 0 VCIDX: 1 (1200 mV)\\n"
        ... "--------------------\\n"
        ... "--------------------\\n"
        ... "Valid config IDX 2\\n"
        ... "[SYSCLK@8000000|-]-->[PLL_R@8000000|/8-]-->[PLL_VCO@64000000|*16-]-->[PLL_M@4000000|/2-]-->[PLL_PREDIV_MUX@8000000|-]-->[MSI@8000000|-]-->[MSIMUX@8000000|-]-->[MSIRANGE@8000000|*80-]-->[MSI_BASE@100000|-]\\n"
        ... "LVPOL| WS: 1 VCIDX: 0 (1000 mV)\\n"
        ... "FFPOL| WS: 0 VCIDX: 1 (1200 mV)\\n"
        ... "--------------------\\n"
        ... "--------------------\\n"
        ... "Valid config IDX 3\\n"
        ... "[SYSCLK@8000000|-]-->[PLL_R@8000000|/8-]-->[PLL_VCO@64000000|*8-]-->[PLL_M@8000000|/2-]-->[PLL_PREDIV_MUX@16000000|-]-->[MSI@16000000|-]-->[MSIMUX@16000000|-]-->[MSIRANGE@16000000|*160-]-->[MSI_BASE@100000|-]\\n"
        ... "LVPOL| WS: 1 VCIDX: 0 (1000 mV)\\n"
        ... "FFPOL| WS: 0 VCIDX: 1 (1200 mV)\\n"
        ... "--------------------\\n")
        >>> len(res['topoconfs'])
        4
        >>> res['clkname']
        'SYSCLK'
        >>> res['maxlen']
        9
        >>> res['target_freq']
        8000000
        >>> res['topoconfs'][3]['clockconfs'][7]['clkname']
        'MSIRANGE'
        >>> res['topoconfs'][3]['clockconfs'][7]['factor']
        160
        >>> res['topoconfs'][3]['clockconfs'][7]['freq']
        16000000
        >>> res['valid_confcnt'] == len(res['topoconfs'])
        True
        """  # noqa: E501

        res = {}
        topo_confs = []
        mdmatches = self.result_metadata_re.finditer(cmd_output)
        for m in mdmatches:
            res.update(self._str_elems_to_int(m.groupdict()))

        matches = self.clock_conf_section_re.finditer(cmd_output)
        for m in matches:
            gd = m.groupdict()
            topo_conf_metadata = self._str_elems_to_int(m.groupdict())

            # res.update(topo_conf_values)
            if "rawConfLine" in gd:
                clock_confs = []
                for cc in gd["rawConfLine"].split("-->"):
                    ccm = self.clock_conf_block_re.match(cc)
                    if ccm:
                        clock_confs.append(self._str_elems_to_int(ccm.groupdict()))
                if clock_confs:
                    topo_conf_metadata["clockconfs"] = clock_confs

            topo_confs.append(topo_conf_metadata)

        res["topoconfs"] = topo_confs
        return res


# ==== ShellInteractions ====
class GCLKManager(ShellInteraction):
    CLOCKMAN = "clockman"
    CLOSEST = "closest"

    @ShellInteraction.check_term
    def clockman_cmd(self, cmd, args=None, timeout=-1, async_=False):
        return self.cmd(
            self._create_cmd(self.CLOCKMAN, cmd, args), timeout=timeout, async_=async_
        )

    @staticmethod
    def _create_cmd(cmd, sub_cmd, args=None):
        cmd_str = f"{cmd} {sub_cmd}"
        if args is not None:
            cmd_str += f" {' '.join(str(a) for a in args)}"
        return cmd_str

    def clockman_closest(
        self,
        clkname,
        targetfreq,
        topoid=None,
        cmpfun=None,
        listall=False,
        timeout=-1,
        async_=False,
    ):
        args = [clkname, targetfreq]
        if topoid:
            args.append(topoid)
            if cmpfun:
                args.append(cmpfun)
                if listall:
                    args.append("listall")
        print(args)
        return self.clockman_cmd(self.CLOSEST, args, timeout, async_)


class GCLKDvfs(ShellInteraction):
    DVFS = "dvfs"
    WSA = "wsa"
    DVS = "dvs"
    DFS = "dfs"

    @ShellInteraction.check_term
    def dvfs_cmd(self, cmd, args=None, timeout=-1, async_=False):
        return self.cmd(
            self._create_cmd(self.DVFS, cmd, args), timeout=timeout, async_=async_
        )

    @staticmethod
    def _create_cmd(cmd, sub_cmd, args=None):
        cmd_str = f"{cmd} {sub_cmd}"
        if args is not None:
            cmd_str += f" {' '.join(str(a) for a in args)}"
        return cmd_str

    def dvfs_dvs_on(self, timeout=-1, async_=False):
        args = [self.DVS, "on"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dvs_off(self, timeout=-1, async_=False):
        args = [self.DVS, "off"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dvs_info(self, timeout=-1, async_=False):
        args = [self.DVS, "info"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dvs_set(self, voltage_idx, timeout=-1, async_=False):
        args = [self.DVS, "set", voltage_idx]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dvs_ffpol(self, timeout=-1, async_=False):
        args = [self.DVS, "ffpol"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dvs_lvpol(self, timeout=-1, async_=False):
        args = [self.DVS, "lvpol"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dfs_on(self, timeout=-1, async_=False):
        args = [self.DFS, "on"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dfs_off(self, timeout=-1, async_=False):
        args = [self.DFS, "off"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dfs_info(self, timeout=-1, async_=False):
        args = [self.DFS, "info"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dfs_set(self, scale_setting_idx, timeout=-1, async_=False):
        args = [self.DFS, "set", scale_setting_idx]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dfs_freqs_get(self, timeout=-1, async_=False):
        args = [self.DFS, "freqs", "get"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_dfs_freqs_set(self, freqs, timeout=-1, async_=False):
        # fallback to default if no list of freqs is given
        freq_vals = " ".join(freqs) if not freqs else "default"
        args = [self.DFS, "freqs", "set", freq_vals]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_wsa_on(self, timeout=-1, async_=False):
        args = [self.WSA, "on"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_wsa_off(self, timeout=-1, async_=False):
        args = [self.WSA, "off"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_wsa_get(self, timeout=-1, async_=False):
        args = [self.WSA, "get"]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)

    def dvfs_wsa_set(self, waitstatecount, timeout=-1, async_=False):
        args = [self.WSA, "set", waitstatecount]
        return self.dvfs_cmd(self.dvfs_cmd, args, timeout, async_)


class GCLKClock(ShellInteraction):
    CLOCK = "clock"
    LIST = "list"
    DESCRIBE = "describe"
    ROOTS = "roots"
    CHILDREN = "children"
    AFFECTED = "affected"
    CONF = "conf"

    @ShellInteraction.check_term
    def clock_cmd(self, cmd, args=None, timeout=-1, async_=False):
        return self.cmd(
            self._create_cmd(self.CLOCK, cmd, args), timeout=timeout, async_=async_
        )

    @staticmethod
    def _create_cmd(cmd, sub_cmd, args=None):
        cmd_str = f"{cmd} {sub_cmd}"
        if args is not None:
            cmd_str += f" {' '.join(str(a) for a in args)}"
        return cmd_str

    def clock_list(self, selector=None, timeout=-1, async_=False):
        opts = [None, "enabled", "disabled"]
        args = []
        if selector in opts:
            if selector:
                args.append(selector)
        else:
            raise ValueError(f"filter must be one of {opts}")
        return self.clock_cmd(self.LIST, args, timeout, async_)

    def clock_describe(self, clock=None, timeout=-1, async_=False):
        args = []
        if clock:
            args.append(clock)

        return self.clock_cmd(self.DESCRIBE, args, timeout, async_)

    def clock_roots(self, clock, timeout=-1, async_=False):
        args = [clock]
        return self.clock_cmd(self.ROOTS, args, timeout, async_)

    def clock_chidren(self, clock, recursive=False, timeout=-1, async_=False):
        recursive_valid_opts = [True, False]
        args = [clock]
        if recursive in recursive_valid_opts and recursive:
            args.append("r")
        else:
            raise ValueError(
                f"recursive parameter must be one of {recursive_valid_opts}"
            )

        return self.clock_cmd(self.CHILDREN, args, timeout, async_)

    def clock_affected(self, altered_clock, affected_clock, timeout=-1, async_=False):
        args = [altered_clock, affected_clock]
        return self.clock_cmd(self.AFFECTED, args, timeout, async_)

    def clock_conf_enable(self, clock, timeout=-1, async_=False):
        args = ["enable", clock]
        return self.clock_cmd(self.CONF, args, timeout, async_)

    def clock_conf_disable(self, clock, timeout=-1, async_=False):
        args = ["disable", clock]
        return self.clock_cmd(self.CONF, args, timeout, async_)

    def clock_conf_get_factor(self, clock, timeout=-1, async_=False):
        args = ["get", "factor", clock]
        return self.clock_cmd(self.CONF, args, timeout, async_)

    def clock_conf_set_factor(
        self, clock, factor, autoreconf=False, timeout=-1, async_=False
    ):
        args = ["set", "factor", clock, factor]
        if autoreconf:
            args.append("autoreconf")
        return self.clock_cmd(self.CONF, args, timeout, async_)

    def clock_conf_get_freq(self, clock, timeout=-1, async_=False):
        args = ["get", "freq", clock]
        return self.clock_cmd(self.CONF, args, timeout, async_)

    def clock_conf_set_freq(
        self, clock, freq, autoreconf=False, timeout=-1, async_=False
    ):
        args = ["set", "freq", clock, freq]
        if autoreconf:
            args.append("autoreconf")
        return self.clock_cmd(self.CONF, args, timeout, async_)

    def clock_conf_get_parent(self, clock, timeout=-1, async_=False):
        args = ["get", "parent", clock]
        return self.clock_cmd(self.CONF, args, timeout, async_)

    def clock_conf_set_parent(
        self, clock, parent, autoreconf=False, timeout=-1, async_=False
    ):
        args = ["set", "parent", clock, parent]
        if autoreconf:
            args.append("autoreconf")
        return self.clock_cmd(self.CONF, args, timeout, async_)
