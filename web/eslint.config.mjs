import nextVitals from 'eslint-config-next/core-web-vitals';
import nextTs from 'eslint-config-next/typescript';

const eslintConfig = [
  ...nextVitals,
  ...nextTs,
  { ignores: ['.next/**', 'out/**', 'node_modules/**', 'public/wasm/**', 'next-env.d.ts'] },
  {
    rules: {
      'no-restricted-syntax': [
        'error',
        { selector: 'ConditionalExpression', message: 'Ternary operators are not allowed; use if/else or lookup objects.' },
      ],
    },
  },
];

export default eslintConfig;
