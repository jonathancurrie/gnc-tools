/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  docsSidebar: [
    {type: 'doc', id: 'index', label: 'Home'},
    {
      type: 'category',
      label: 'GNC overview',
      collapsed: true,
      items: [
        'overview/what-is-gnc',
        'overview/what-gnc-engineers-do',
        'overview/nomenclature',
      ],
    },
    {
      type: 'category',
      label: 'PID control',
      collapsed: true,
      link: {type: 'doc', id: 'control/pid/index'},
      items: [
        {
          type: 'doc',
          id: 'control/pid/controller-types',
          label: 'P, PI, PD and PID',
        },
        {type: 'doc', id: 'control/pid/anti-windup', label: 'Anti WindUp'},
        {
          type: 'doc',
          id: 'control/pid/process-variable-derivative',
          label: 'Process Variable Derivative',
        },
        {
          type: 'doc',
          id: 'control/pid/derivative-filter',
          label: 'Derivative Filter',
        },
        {
          type: 'doc',
          id: 'control/pid/setpoint-weighting',
          label: 'Setpoint Weighting',
        },
        {
          type: 'doc',
          id: 'control/pid/setpoint-ramping',
          label: 'Setpoint Ramping',
        },
        {type: 'doc', id: 'control/pid/tuning', label: 'Tuning Guide'},
        {type: 'doc', id: 'control/pid/stability', label: 'Stability Guide'},
      ],
    },
    {
      type: 'category',
      label: 'Digital filtering',
      collapsed: true,
      link: {type: 'doc', id: 'control/filtering/index'},
      items: [
        {type: 'doc', id: 'control/filtering/aliasing', label: 'Aliasing'},
        'control/filtering/filter-design',
        {type: 'doc', id: 'control/filtering/software', label: 'Software'},
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      collapsed: true,
      items: [
        'resources/software',
        'resources/space-news-sites',
        'resources/bibliography',
      ],
    },
  ],
};

module.exports = sidebars;
