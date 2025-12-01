import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'From Unboxed to Inspecting in Minutes',
    Svg: require('@site/static/img/vision_ai.svg').default,
    description: (
      <>
        Plug in your inspection device, follow the quick-start guide, and run
        your first inspection in just a few steps. These docs walk you through
        setup, calibration, and verification so you can go from hardware on the
        bench to production-ready checks fast.
      </>
    ),
  },
  {
    title: 'Built for Real Manufacturing Workflows',
    Svg: require('@site/static/img/factory_workflow.svg').default,
    description: (
      <>
        Use the documentation to model your actual line: stations, recipes,
        projects, and versions. We focus on practical examples and clear
        workflows so engineers, technicians, and operators can all find exactly
        what they need quickly.
      </>
    ),
  },
  {
    title: 'AI-Powered Vision, Fully Extensible',
    Svg: require('@site/static/img/extensible_system.svg').default,
    description: (
      <>
        The platform uses modern computer vision and AI. These docs show how to
        extend the system with custom rules, APIs, and integrations—while
        maintaining consistent UI, logging, and deployment across all inspection
        devices.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
