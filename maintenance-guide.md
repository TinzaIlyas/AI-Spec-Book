# Book Update & Maintenance Guide for Physical AI & Humanoid Robotics

## Overview

Ye guide aapko comprehensive instructions provide karegi ke "Physical AI & Humanoid Robotics" book ko maintain aur update kaise karna hai, including both the Docusaurus website aur print versions.

## Adding New Chapters

### Step 1: Create the New Chapter File

1. Navigate to the appropriate module directory in `docs/`
2. Create a new markdown file with a descriptive name:
   ```bash
   # Example for a new chapter in Module 2
   touch docs/module2-new-ai-technique.md
   ```

3. Add proper frontmatter to the new chapter:
   ```markdown
   ---
   sidebar_position: 2  # Adjust based on position in module
   title: "Module 2: New AI Technique"
   ---

   # Module 2: New AI Technique

   ## Overview

   Ye chapter new AI technique ko cover karegi jo robotics mein use hoti hai...
   ```

### Step 2: Update the Sidebar Configuration

1. Open `sidebars.ts` file
2. Add the new chapter to the appropriate module category:
   ```javascript
   {
     type: 'category',
     label: 'Module 2: Advanced AI Techniques',
     items: [
       'module2-machine-learning',
       'module2-new-ai-technique',  // Add this line
       'module2-computer-vision',
       // ... other items
     ],
   },
   ```

### Step 3: Update Table of Contents

If creating a completely new module:
1. Add a new category in `sidebars.ts`
2. Ensure proper numbering and positioning
3. Update any overview chapters to reference the new content

## Updating Existing Content

### Content Updates

1. Edit the specific markdown file directly
2. Preserve the frontmatter structure
3. Test changes using `npm start` in the Docusaurus environment
4. Verify code blocks and formatting remain intact

### Structure Updates

1. If reorganizing chapters, update `sidebars.ts` accordingly
2. Update any internal links between chapters
3. Ensure navigation flow remains logical

## Docusaurus Website Maintenance

### Regular Maintenance Tasks

#### 1. Dependency Updates
```bash
# Check for outdated packages
npm outdated

# Update packages (carefully, test after updates)
npm update

# Update to latest Docusaurus version
npm install @docusaurus/core@latest @docusaurus/preset-classic@latest
```

#### 2. Build Testing
```bash
# Test the build process
npm run build

# Serve the built site locally
npm run serve
```

#### 3. Link Checking
```bash
# Install link checker
npm install -g markdown-link-check

# Check links in all markdown files
find docs -name "*.md" -exec markdown-link-check {} \;
```

### Performance Optimization

1. Optimize images before adding
2. Minimize large code blocks
3. Use lazy loading for heavy components
4. Regularly clean up unused files

## PDF Version Maintenance

### Regenerating PDF After Updates

1. After content updates, regenerate the PDF using the same commands:
   ```bash
   # Windows
   pandoc docs/intro.md docs/module1-introduction.md docs/module2-machine-learning.md docs/module3-bipedal-locomotion.md docs/module4-sensor-fusion.md docs/capstone-design-challenge.md docs/hardware-selection.md docs/deployment-guide.md -o "Physical_AI_Humanoid_Robotics_v1.1.pdf" --pdf-engine=xelatex --template=templates/roman-urdu-template.tex --metadata-file=metadata.yaml --toc --toc-depth=2 --number-sections --variable=geometry:"a4paper,margin=1in" --variable=mainfont:"Times New Roman"
   ```

2. Update the version number in the filename
3. Verify the new PDF renders correctly
4. Check that new content appears properly formatted

### Handling Large Books

For books with many chapters, consider:
1. Creating separate PDFs for each module
2. Using a script to automatically generate the pandoc command
3. Implementing automated PDF generation in your workflow

## GitHub Workflow for Updates

### Branch Strategy

1. **Main branch**: Production-ready content
2. **Development branch**: Active content development
3. **Feature branches**: Individual chapter updates

### Workflow Process

```bash
# 1. Create a feature branch for new content
git checkout -b feature/new-chapter-module3

# 2. Add or update content
# Edit your markdown files

# 3. Test changes locally
npm start

# 4. Commit changes with descriptive message
git add .
git commit -m "Add: New chapter on advanced locomotion techniques in Module 3"

# 5. Push to GitHub
git push origin feature/new-chapter-module3

# 6. Create pull request to development branch
# Review and merge after approval

# 7. When ready for production, merge to main
git checkout main
git merge development
git push origin main
```

### Automated Build Process

Create a GitHub Actions workflow for automated builds:

```yaml
# .github/workflows/build.yml
name: Build and Deploy

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - name: Setup Node.js
      uses: actions/setup-node@v3
      with:
        node-version: '18'
        
    - name: Install dependencies
      run: npm install
      
    - name: Build website
      run: npm run build
      
    - name: Deploy to GitHub Pages
      if: github.ref == 'refs/heads/main'
      run: |
        git remote set-url origin https://github.com/${{ github.repository }}.git
        npm run deploy
      env:
        GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
```

## Quality Assurance

### Pre-Publication Checklist

- [ ] All Roman Urdu text renders correctly
- [ ] Code blocks maintain proper syntax highlighting
- [ ] Internal links function properly
- [ ] Images display at appropriate resolution
- [ ] Navigation flows logically
- [ ] Mobile responsiveness verified
- [ ] PDF version renders correctly
- [ ] All metadata is updated

### Content Review Process

1. **Technical Review**: Verify accuracy of technical content
2. **Language Review**: Check Roman Urdu transliteration consistency
3. **Formatting Review**: Ensure consistent styling
4. **Accessibility Review**: Verify screen reader compatibility

## Backup and Recovery

### Regular Backups

1. GitHub serves as primary backup for source content
2. Maintain local backups of important versions
3. Archive PDF versions for reference
4. Keep template files separately backed up

### Recovery Process

1. Clone the repository if local copy is lost
2. Install dependencies: `npm install`
3. Start the development server: `npm start`

## Troubleshooting Common Issues

### Roman Urdu Rendering Issues
- Verify font support in PDF generation
- Check character encoding (should be UTF-8)
- Ensure proper LaTeX packages are installed

### Docusaurus Build Issues
- Clear cache: `npm run clear`
- Reinstall dependencies: `rm -rf node_modules && npm install`
- Check for conflicting dependencies

### PDF Generation Issues
- Verify LaTeX installation is complete
- Check template file path and permissions
- Ensure all referenced files exist

## Performance Monitoring

### Website Performance
- Monitor page load times
- Check search functionality
- Verify mobile responsiveness
- Track user engagement metrics

### Content Performance
- Monitor which chapters are most accessed
- Collect user feedback
- Track reported issues
- Plan content updates based on usage

This maintenance guide ensures your "Physical AI & Humanoid Robotics" book remains current, accurate, and accessible to readers in both digital and print formats.