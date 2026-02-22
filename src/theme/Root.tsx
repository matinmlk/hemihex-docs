import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

type Props = {
  children: React.ReactNode;
};

export default function Root({ children }: Props): React.JSX.Element {
  const location = useLocation();

  useEffect(() => {
    const body = document.body;
    body.classList.remove('hh-page-leave');
    body.classList.add('hh-page-enter');
    const t = window.setTimeout(() => body.classList.remove('hh-page-enter'), 260);
    return () => window.clearTimeout(t);
  }, [location.pathname, location.search, location.hash]);

  useEffect(() => {
    const onClick = (ev: MouseEvent) => {
      if (ev.defaultPrevented || ev.button !== 0 || ev.metaKey || ev.ctrlKey || ev.shiftKey || ev.altKey) {
        return;
      }

      const target = ev.target as Element | null;
      const link = target?.closest('a[href]') as HTMLAnchorElement | null;
      if (!link) return;

      const href = link.getAttribute('href');
      if (!href || href.startsWith('#') || href.startsWith('mailto:') || href.startsWith('tel:')) return;
      if (link.target === '_blank' || link.hasAttribute('download')) return;

      let url: URL;
      try {
        url = new URL(link.href, window.location.href);
      } catch {
        return;
      }

      if (url.origin !== window.location.origin) return;
      if (url.pathname === window.location.pathname && url.search === window.location.search) return;

      document.body.classList.add('hh-page-leave');
      window.setTimeout(() => document.body.classList.remove('hh-page-leave'), 180);
    };

    document.addEventListener('click', onClick, true);
    return () => document.removeEventListener('click', onClick, true);
  }, []);

  return <>{children}</>;
}
